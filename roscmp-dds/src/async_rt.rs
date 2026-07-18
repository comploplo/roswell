//! Tokio adapter (`tokio` feature): async subscription [`Stream`]s and async
//! service calls over the existing blocking primitives.
//!
//! Bridge mechanism: one plain OS thread per endpoint, pumping into a tokio
//! channel. RustDDS exposes data-availability only as a `mio 0.8` event
//! *source* (not a raw fd), so `AsyncFd` cannot be used portably; instead the
//! pump thread blocks on its own `mio::Poll` waitset — exactly what the
//! synchronous executor does — and forwards decoded samples. The tokio side is
//! a channel receiver: no executor rewrite, correct on macOS and Linux alike.
//! Waits are bounded (100 ms) so a dropped receiver reaps its thread promptly
//! and a missed readiness edge can never wedge the stream.

use std::task::{Context, Poll};
use std::time::Duration;

use tokio::sync::{mpsc, oneshot};

use crate::codec::CdrMsg;
use crate::service::Client;
use crate::transport::{Dds, DdsSub, MsgSubscriber, Qos, Transport};

/// How long a pump thread blocks in `mio::Poll::poll` before re-checking for a
/// dropped receiver. Bounds both shutdown latency and missed-edge recovery.
const PUMP_TICK: Duration = Duration::from_millis(100);

// SAFETY: an owned `RosString`/`RosSequence` is a unique heap allocation (same
// shape as `Vec<u8>`), which is `Send`. A *borrowed* value (capacity == 0) was
// built via the unsafe `from_raw_parts`, whose contract already obliges the
// caller to keep the backing alive for every read — a thread-agnostic
// obligation, so moving the value across threads adds no new hazard.
unsafe impl Send for crate::msgs::RosString {}
unsafe impl<T: Send> Send for crate::msgs::RosSequence<T> {}

/// An async stream of decoded messages from one subscription.
///
/// Implements [`futures_core::Stream`]; `next()` is also provided directly so
/// plain `while let Some(msg) = stream.next().await` works without an adapter
/// crate. Dropping the stream stops the pump thread within [`PUMP_TICK`].
pub struct MsgStream<M> {
    rx: mpsc::UnboundedReceiver<M>,
}

impl<M> MsgStream<M> {
    /// Await the next message; `None` once the pump thread has exited.
    pub async fn next(&mut self) -> Option<M> {
        self.rx.recv().await
    }
}

impl<M> futures_core::Stream for MsgStream<M> {
    type Item = M;

    fn poll_next(mut self: std::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<M>> {
        self.rx.poll_recv(cx)
    }
}

/// Subscribe to `ros_topic` and receive decoded messages as an async stream.
pub fn subscribe<M: CdrMsg + Send>(dds: &Dds, ros_topic: &str, qos: Qos) -> MsgStream<M> {
    let sub = dds.subscriber::<M>(ros_topic, qos);
    let (tx, rx) = mpsc::unbounded_channel();
    std::thread::spawn(move || pump(sub, &tx));
    MsgStream { rx }
}

/// Pump thread body: block on the reader's mio event source, drain every
/// available sample into the channel, exit when the receiver is gone.
fn pump<M: CdrMsg>(mut sub: DdsSub<M>, tx: &mpsc::UnboundedSender<M>) {
    let mut poll = mio::Poll::new().expect("mio poll");
    let mut events = mio::Events::with_capacity(4);
    poll.registry()
        .register(sub.event_source(), mio::Token(0), mio::Interest::READABLE)
        .expect("register reader");
    while !tx.is_closed() {
        // Always drain after waking: draining on every tick (not only on a
        // readiness event) makes a missed edge cost at most one PUMP_TICK.
        let _ = poll.poll(&mut events, Some(PUMP_TICK));
        while let Some(msg) = sub.take() {
            if tx.send(msg).is_err() {
                return;
            }
        }
    }
}

/// One in-flight service call: request + reply slot.
type Pending<Req, Resp> = (Req, Duration, oneshot::Sender<Option<Resp>>);

enum Cmd<Req, Resp> {
    Call(Pending<Req, Resp>),
    Ready(oneshot::Sender<bool>),
}

/// Async wrapper around [`Client`]: the blocking client lives on a dedicated
/// thread and calls are relayed over channels, so an `.await`ed call never
/// blocks the tokio runtime and cancelling (dropping) a call future is safe —
/// the client itself is never lost mid-flight. Dropping the `AsyncClient`
/// closes the command channel and reaps the thread.
pub struct AsyncClient<Req: CdrMsg + Send, Resp: CdrMsg + Send> {
    tx: mpsc::UnboundedSender<Cmd<Req, Resp>>,
}

impl<Req: CdrMsg + Send, Resp: CdrMsg + Send> AsyncClient<Req, Resp> {
    /// Bind an async client to `/<service>` on `dds`.
    #[must_use]
    pub fn new(dds: &Dds, service: &str) -> Self {
        let mut client = Client::<Req, Resp>::new(dds, service);
        let (tx, mut rx) = mpsc::unbounded_channel::<Cmd<Req, Resp>>();
        std::thread::spawn(move || {
            while let Some(cmd) = rx.blocking_recv() {
                match cmd {
                    Cmd::Call((req, timeout, reply)) => {
                        let _ = reply.send(client.call(req, timeout));
                    }
                    Cmd::Ready(reply) => {
                        let _ = reply.send(client.server_is_ready());
                    }
                }
            }
        });
        Self { tx }
    }

    /// True once a server is discovered end to end (see
    /// [`Client::server_is_ready`]). Await this before the first call.
    pub async fn server_is_ready(&self) -> bool {
        let (reply_tx, reply_rx) = oneshot::channel();
        if self.tx.send(Cmd::Ready(reply_tx)).is_err() {
            return false;
        }
        reply_rx.await.unwrap_or(false)
    }

    /// Send `req` and await the correlated reply, up to `timeout`. Returns
    /// `None` on timeout (or if the client thread has exited).
    pub async fn call(&self, req: Req, timeout: Duration) -> Option<Resp> {
        let (reply_tx, reply_rx) = oneshot::channel();
        self.tx.send(Cmd::Call((req, timeout, reply_tx))).ok()?;
        reply_rx.await.ok()?
    }
}
