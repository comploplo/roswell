//! Generation-counted handle table — the memory-safety core of the C ABI.
//!
//! Every object the FFI hands out is addressed by an opaque [`RcmHandle`] (a
//! `u64` packing a 32-bit slot index and a 32-bit generation counter) rather
//! than a raw pointer. Freeing a handle bumps its slot's generation, so a stale
//! handle — use-after-free, double-free, use-after-shutdown — validates to a
//! distinct error ([`HandleError::Stale`]) instead of dereferencing dangling
//! memory. Each slot also records its [`Kind`], so using a handle of the wrong
//! kind is a clean [`HandleError::WrongKind`], never a type-confused transmute.
//!
//! # Concurrency
//! The table is a single process-global `Mutex`. A lookup clones out the
//! entry's `Arc<Mutex<Payload>>` while holding the table lock, then releases it
//! immediately — the expensive work (encode/decode, DDS I/O) runs holding only
//! the per-object lock, so a background `rcm_wait` thread and a user
//! `rcm_publish` thread contend on the table only for the microseconds of
//! validation. Because a live operation holds an `Arc` clone, a concurrent
//! `rcm_shutdown` that drops the table's clone cannot free the object out from
//! under it: the worst case is operating on a detached object, never UB. See
//! the "Threading" section of `include/roswell.h`.

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use crate::{
    RcmActionClient, RcmActionServer, RcmBagReader, RcmBagWriter, RcmClient, RcmCtx, RcmMsg,
    RcmParamServer, RcmPublisher, RcmService, RcmSubscriber, RcmTf, RcmType,
};

/// Opaque handle: `(generation << 32) | slot`. `0` is never valid (generations
/// start at 1), so it doubles as the null/error sentinel across the FFI.
pub type RcmHandle = u64;

/// The kind of object a handle refers to. Validated on every lookup so a handle
/// of one kind can never be used where another is expected.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    Ctx,
    Type,
    Publisher,
    Subscriber,
    Service,
    Client,
    Msg,
    ParamServer,
    ActionClient,
    ActionServer,
    BagWriter,
    BagReader,
    Tf,
}

/// The owned object behind a handle.
pub enum Payload {
    Ctx(RcmCtx),
    Type(RcmType),
    Publisher(RcmPublisher),
    Subscriber(RcmSubscriber),
    Service(RcmService),
    Client(RcmClient),
    Msg(RcmMsg),
    ParamServer(RcmParamServer),
    // Boxed: an action client bundles five dynamic types and four DDS endpoints,
    // far larger than any other variant.
    ActionClient(Box<RcmActionClient>),
    // Boxed for the same reason: eight dynamic types plus five DDS endpoints.
    ActionServer(Box<RcmActionServer>),
    BagWriter(RcmBagWriter),
    BagReader(RcmBagReader),
    Tf(Box<RcmTf>),
}

impl Payload {
    fn kind(&self) -> Kind {
        match self {
            Payload::Ctx(_) => Kind::Ctx,
            Payload::Type(_) => Kind::Type,
            Payload::Publisher(_) => Kind::Publisher,
            Payload::Subscriber(_) => Kind::Subscriber,
            Payload::Service(_) => Kind::Service,
            Payload::Client(_) => Kind::Client,
            Payload::Msg(_) => Kind::Msg,
            Payload::ParamServer(_) => Kind::ParamServer,
            Payload::ActionClient(_) => Kind::ActionClient,
            Payload::ActionServer(_) => Kind::ActionServer,
            Payload::BagWriter(_) => Kind::BagWriter,
            Payload::BagReader(_) => Kind::BagReader,
            Payload::Tf(_) => Kind::Tf,
        }
    }
}

/// Why a handle failed validation. Maps 1:1 to the `RCM_ERR_*` codes in the
/// header.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HandleError {
    /// The handle was `0` (the null sentinel).
    Null,
    /// The slot is free or its generation no longer matches: the object was
    /// freed, double-freed, or invalidated by `rcm_shutdown`.
    Stale,
    /// The slot is live but holds a different [`Kind`] than expected.
    WrongKind,
}

struct Live {
    kind: Kind,
    /// Owning context handle, for cascade invalidation on `rcm_shutdown`.
    /// `None` for context-independent objects (types and messages).
    parent: Option<RcmHandle>,
    obj: Arc<Mutex<Payload>>,
}

struct Slot {
    generation: u32,
    live: Option<Live>,
}

/// A slab of generation-counted slots plus a free list of reusable indices.
pub struct HandleTable {
    slots: Vec<Slot>,
    free: Vec<u32>,
}

fn encode(slot: u32, generation: u32) -> RcmHandle {
    (u64::from(generation) << 32) | u64::from(slot)
}

fn decode(handle: RcmHandle) -> (u32, u32) {
    ((handle & 0xFFFF_FFFF) as u32, (handle >> 32) as u32)
}

impl HandleTable {
    const fn new() -> Self {
        Self {
            slots: Vec::new(),
            free: Vec::new(),
        }
    }

    fn insert(&mut self, parent: Option<RcmHandle>, payload: Payload) -> RcmHandle {
        let kind = payload.kind();
        let live = Live {
            kind,
            parent,
            obj: Arc::new(Mutex::new(payload)),
        };
        if let Some(slot) = self.free.pop() {
            let s = &mut self.slots[slot as usize];
            s.live = Some(live);
            encode(slot, s.generation)
        } else {
            let slot = u32::try_from(self.slots.len()).expect("handle table exhausted");
            self.slots.push(Slot {
                generation: 1,
                live: Some(live),
            });
            encode(slot, 1)
        }
    }

    fn lookup(&self, handle: RcmHandle, kind: Kind) -> Result<Arc<Mutex<Payload>>, HandleError> {
        if handle == 0 {
            return Err(HandleError::Null);
        }
        let (slot, generation) = decode(handle);
        let s = self.slots.get(slot as usize).ok_or(HandleError::Stale)?;
        if s.generation != generation {
            return Err(HandleError::Stale);
        }
        let live = s.live.as_ref().ok_or(HandleError::Stale)?;
        if live.kind != kind {
            return Err(HandleError::WrongKind);
        }
        Ok(Arc::clone(&live.obj))
    }

    fn remove(
        &mut self,
        handle: RcmHandle,
        kind: Kind,
    ) -> Result<Arc<Mutex<Payload>>, HandleError> {
        if handle == 0 {
            return Err(HandleError::Null);
        }
        let (slot, generation) = decode(handle);
        let s = self
            .slots
            .get_mut(slot as usize)
            .ok_or(HandleError::Stale)?;
        if s.generation != generation {
            return Err(HandleError::Stale);
        }
        match &s.live {
            Some(live) if live.kind != kind => return Err(HandleError::WrongKind),
            Some(_) => {}
            None => return Err(HandleError::Stale),
        }
        let live = s.live.take().expect("checked live above");
        bump(&mut s.generation);
        self.free.push(slot);
        Ok(live.obj)
    }

    fn invalidate_children(&mut self, ctx: RcmHandle) {
        for i in 0..self.slots.len() {
            let is_child = self.slots[i]
                .live
                .as_ref()
                .is_some_and(|l| l.parent == Some(ctx));
            if is_child {
                self.slots[i].live = None;
                bump(&mut self.slots[i].generation);
                self.free
                    .push(u32::try_from(i).expect("slot index fits u32"));
            }
        }
    }
}

/// Advance a generation, skipping 0 so a reused slot never produces the null
/// handle and a wrapped counter never collides with the fresh-slot value.
fn bump(generation: &mut u32) {
    *generation = generation.wrapping_add(1);
    if *generation == 0 {
        *generation = 1;
    }
}

static TABLE: Mutex<HandleTable> = Mutex::new(HandleTable::new());

/// Lock the global table, recovering from a poisoned lock rather than
/// propagating a panic across the FFI boundary.
fn table() -> MutexGuard<'static, HandleTable> {
    TABLE.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Register `payload` and return its fresh handle. `parent` is the owning
/// context (for cascade invalidation) or `None` for types and messages.
pub fn insert(parent: Option<RcmHandle>, payload: Payload) -> RcmHandle {
    table().insert(parent, payload)
}

/// Validate `handle` against `kind` and clone out its object for use. The table
/// lock is released before the returned `Arc` is locked.
pub fn lookup(handle: RcmHandle, kind: Kind) -> Result<Arc<Mutex<Payload>>, HandleError> {
    table().lookup(handle, kind)
}

/// Validate and remove `handle`, bumping its slot's generation so every
/// outstanding copy of it becomes [`HandleError::Stale`]. Returns the object so
/// the caller can run any teardown (e.g. finalizing a message) before it drops.
pub fn remove(handle: RcmHandle, kind: Kind) -> Result<Arc<Mutex<Payload>>, HandleError> {
    table().remove(handle, kind)
}

/// Invalidate every handle whose parent is `ctx` (its publishers, subscribers,
/// services, and clients). Called by `rcm_shutdown`.
pub fn invalidate_children(ctx: RcmHandle) {
    table().invalidate_children(ctx);
}

/// Lock a looked-up object, recovering from poisoning.
pub fn lock(obj: &Arc<Mutex<Payload>>) -> MutexGuard<'_, Payload> {
    obj.lock().unwrap_or_else(PoisonError::into_inner)
}
