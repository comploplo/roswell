# Real-time properties of roscmp's hot paths

This document reports two things, honestly separated:

1. **A hot-path audit** (Part A) — where the publish/subscribe/spin/tunnel paths
   allocate, loop, recurse, panic, block, and where roscmp's code ends and
   RustDDS's begins.
2. **Machine-checked proofs** (Part B) — the pure arithmetic cores of those
   paths carry Creusot contracts discharged by Why3/SMT. The claims here are
   exactly as strong as the proofs, and no stronger.

Reproduce the proofs with `scripts/verify-rt.sh` (requirements documented at the
top of that script). The normal build and test gate (`scripts/check.sh`) is
untouched by any of this: Creusot is opt-in behind the `verify` feature and the
`--cfg creusot` flag, and `creusot-std` is never compiled into a normal build.

---

## Part A — Hot-path audit

Scope: roscmp's own code. **RustDDS is out of scope and unaudited**; the
boundary is called out for each path. "Bounded" below means bounded by message
size / configured capacity, not constant-time.

### 1. Publish path: `dynamic::encode` → raw publish

`DynamicType::encode` (`src/dynamic.rs:261`) drives `Writer` over the C-ABI
struct.

- **Allocations.** One `Vec` in `Writer::new` (`src/cdr_runtime.rs:64`,
  `with_capacity(64)`) that grows by `push`/`extend`/`resize` as the body is
  written — so a large message triggers repeated reallocation (not pre-sized to
  the final length). Total allocation is bounded by the encoded size.
- **Recursion.** `encode_message → encode_element → encode_message` for nested
  messages (`src/dynamic.rs:267,299,307`). Depth = static message-nesting depth.
  `resolve::topo_order` rejects by-value cycles (`src/resolve.rs:296`), so the
  by-value closure is a finite DAG; recursion is bounded by the message
  definition, not by input. **No unbounded recursion.**
- **Loops.** `for i in 0..len` over fixed arrays (`src/dynamic.rs:275`) and
  `for i in 0..size` over sequences (`src/dynamic.rs:287`), where `size` is read
  from the in-memory triple. Bounded by the struct's own contents.
- **Panics reachable from release.** `self.layouts[id]` index
  (`src/dynamic.rs:269`) panics if a message id is missing — but the closure is
  complete after construction, so this is unreachable in practice. `read_ros_str`
  uses `from_utf8_unchecked` (`src/dynamic.rs:640`): a caller-supplied non-UTF-8
  string is UB, not a panic (documented `# Safety`).
- **Locks / blocking.** None in the encode itself.
- **`unsafe`.** The whole module reads caller memory via `read_unaligned`; encode
  is a read-only pass over the message struct.
- **RustDDS boundary.** `RawDdsPublisher::publish` hands the CDR bytes to
  `self.writer.write(...)` (`roscmp-dds/src/raw.rs:668`). Everything past that
  call — serialization copies, history cache, RTPS, sockets, internal locks,
  blocking under `max_blocking_time` — is **RustDDS, out of scope.**

### 2. Subscribe path: take → `dynamic::decode`

`RawDdsSubscriber::take` (`roscmp-dds/src/raw.rs:732`) pulls a sample from
RustDDS, then `DynamicType::decode` (`src/dynamic.rs:324`) writes it into caller
memory.

- **Allocations.**
  - ~~Per nested message, the layout is cloned~~ **FIXED**: `decode_message`
    now borrows the layout (`self.layout_of(id)?`); the clone was unnecessary
    (all borrows involved are immutable). No per-message layout allocation
    remains on the decode path.
  - `alloc_buf` per sequence (`src/dynamic.rs:348`) and `store_ros_string` per
    string (`src/dynamic.rs:642`). Bounded by message content.
- ~~Attacker-controlled loop bound~~ **FIXED at the root**:
  `Reader::read_seq_len` (`src/cdr_runtime.rs`) now rejects any element count
  exceeding the bytes remaining in the buffer (every element occupies ≥ 1 wire
  byte — even an empty nested message encodes a dummy octet), returning
  `Truncated` before any allocation. The guard covers the dynamic codec, the
  hand-rolled message modules, and the generated runtime alike (the generated
  preamble embeds `cdr_runtime.rs` verbatim; `msgs.rs` regenerated). A
  malformed prefix is now bounded to a `remaining-bytes`-sized allocation at
  worst.
- **Recursion / RustDDS boundary.** Same nesting bound as encode; the boundary
  is `reader.take_next_sample()` (`roscmp-dds/src/raw.rs:734`).

### 3. `node::spin_once` tick

`Node::spin_once` (`roscmp-dds/src/node.rs:330`): `loop { drain(); … sleep }`.

- **Blocking.** `std::thread::sleep(...)` (`roscmp-dds/src/node.rs:338`) — the
  executor blocks up to the caller's timeout by design.
- **`drain`** (`node.rs:308`) iterates existing timer/subscription/service Vecs;
  no per-tick allocation of its own (downstream `sub.poll()`/`service.serve()`
  decode and thus allocate).
- **Timer catch-up.** `Timer::fire` (`node.rs`) advances the deadline via the
  **proven** `roscmp_verify::next_fire_after` (see Part B). The clock fallbacks
  now saturate to `i64::MAX - 1`, so the proof's `now < i64::MAX` precondition
  holds at every call site.
- **Locks.** Shutdown is an `AtomicBool` (`node.rs:359,364`), no mutex.

### 4. `tunnel` enqueue / dequeue / policy

- **Bounded queue.** `OutboundQueue::enqueue` (`roscmp-dds/src/tunnel.rs:677`)
  admits frames through the **proven** `roscmp_verify::plan_enqueue` drop policy
  (see Part B); `pop_next` is O(1) across three priority `VecDeque`s. Backlog is
  bounded by `max_pending` per channel.
- ~~Unbounded growth~~ **FIXED**: `TopicBridgeRx::received_sequences` is now a
  `BTreeSet<u64>` sliding window capped at `DEDUP_WINDOW = 4096` entries
  (oldest sequence evicted). Memory is bounded for indefinite uptime; a
  duplicate arriving more than a window late would re-publish, which the
  near-term retry design makes practically unreachable.
- **Locks.** `TunnelReliabilityHandle` is `Arc<Mutex<…>>`; the TX/RX paths take
  `.lock().expect("… poisoned")` (`tunnel.rs:319…`). Uncontended fast, but a
  poisoned lock **panics**, and a contended lock blocks.
- **Frame I/O DoS bound.** Inbound frames are length-prefixed and rejected above
  `MAX_FRAME_LEN = 64 MiB` (`tunnel.rs:18`) before `vec![0; len]` allocates, so a
  hostile length field cannot request an unbounded buffer.
- **Time.** `now_system_nanos` saturates (`i64::try_from(...).unwrap_or(i64::MAX)`),
  no panic.

---

## Part B — Machine-checked properties

The pure arithmetic cores of the paths above live in the dependency-free
**`roscmp-verify`** crate and are the *production* implementations the hot paths
call — not parallel copies. They are isolated there only because Creusot cannot
translate the surrounding crates (recursive IR enums in `ast`/`idl`, `unsafe`
pointer code in `dynamic`, RustDDS, nom); the small pure crate translates in
full. Contracts are erased in normal builds.

Verified with cargo-creusot 0.13.0-dev, toolchain nightly-2026-06-22, Why3
1.8.2, why3find 1.3.0, provers alt-ergo 2.6.2 / z3 4.15.3 / cvc5 1.3.1 / cvc4
1.8. Each is discharged (`Proved (4 files) ✔`) and the proof sessions are
checked in.

### Proven properties

| Function (`roscmp-verify`) | Production caller | Property proven | Artifact |
|---|---|---|---|
| `pad_to(off, a)` | CDR `Writer`/`Reader` alignment (`src/cdr_runtime.rs`, pinned by equivalence test) | `a>0` ⇒ no panic (no div-by-zero, no underflow in `a-off%a`); `result < a`; `(off+result) % a == 0` (correct alignment) | `verif/roscmp_verify_rlib/pad_to/proof.json` |
| `round_up(off, align)` | `dynamic::round_up` layout offset rounding (`src/dynamic.rs:594`) | `align>0` ∧ `off+align ≤ usize::MAX` ⇒ no overflow; `off ≤ result < off+align`; `result % align == 0` | `verif/roscmp_verify_rlib/round_up/proof.json` |
| `next_fire_after(next_fire, period, now)` | `Timer::fire` catch-up (`roscmp-dds/src/node.rs:733`) | `period≥1` ∧ `now<i64::MAX` ⇒ **loop terminates** (variant `now-cur`); `result ≥ next_fire` (deadline never rewinds); `result > now` | `verif/roscmp_verify_rlib/next_fire_after/proof.json` |
| `plan_enqueue(len, max_pending, drop_oldest)` | `OutboundQueue::enqueue` drop policy (`roscmp-dds/src/tunnel.rs:693`) | `max_pending≥1` ⇒ resulting length never rises above `max_pending` unless already above it; a *net* enqueue happens only with strict room; never evicts from an empty lane | `verif/roscmp_verify_rlib/plan_enqueue/proof.json` |

"No panic" = Creusot's automatic safety VCs (integer overflow/underflow,
division by zero, indexing) are discharged in addition to the functional
contract.

The CDR `pad_to` is embedded verbatim into generated bindings, so it cannot call
an external crate; `cdr::tests::pad_to_matches_verified_core` (`src/cdr.rs`)
pins the embedded formula to the verified `roscmp_verify::pad_to` over every CDR
alignment and a wide offset range, transferring the proof to that copy.

### A bug the proof surfaced

`next_fire_after` terminates **only** for `now < i64::MAX`. The clock fallback
`tick_nanos → unwrap_or(i64::MAX)` (`node.rs`) could, in the ~292-year-past-epoch
/ clock-failure corner, feed `now == i64::MAX`, at which the original
`while next_fire_nanos <= now` loop would spin forever. The precondition named
that corner precisely; both fallbacks now saturate to `i64::MAX - 1`, closing
it at every call site.

---

## What is NOT proven (and cannot be, with this approach)

These proofs are about **arithmetic and control-flow correctness**, not timing.
None of the following is established here:

- **No WCET / no timing bound.** Creusot proves *termination*, never *how long*.
  There is no worst-case execution time, no cycle count, no deadline guarantee.
- **Allocator behaviour.** The publish/subscribe paths call the global allocator
  (`Vec` growth, `alloc_buf`, layout clones, string copies). Allocation *latency*
  and fragmentation are unmodeled; a real RT deployment would need a bounded /
  real-time allocator and pre-sizing, neither proven here.
- **The full encode/decode walk is not proven** — only the arithmetic cores are.
  The `unsafe` pointer walk, UTF-8 assumptions, and the malformed-input DoS panic
  in `alloc_buf` (Part A §2) are audited, not verified.
- **RustDDS internals** — serialization, history caches, RTPS, sockets, locks,
  blocking under QoS — are entirely out of scope and unverified.
- **OS scheduling, priority inversion, interrupt latency, `thread::sleep`
  accuracy** — all unmodeled.
- **Concurrency.** The `Arc<Mutex<TunnelReliabilityState>>` path and the
  `received_sequences` unbounded growth (Part A §4) are audit findings, not
  proofs; no lock-freedom or deadlock-freedom is claimed.

## Honest claims these proofs support

Supported:

- "The CDR **alignment arithmetic** is machine-checked panic-free and correct
  (Creusot/Why3): padding is always `< alignment` and lands the cursor on an
  exact multiple."
- "Layout **offset rounding** is machine-checked overflow-free and correct."
- "The **timer catch-up** is machine-checked to terminate and to never rewind a
  deadline (given a nanosecond clock below the i64 ceiling)."
- "The **tunnel drop policy** is machine-checked to keep a channel's backlog
  within its configured `max_pending`."
- "These are the production code paths, not a separate model, and the proofs are
  reproducible (`scripts/verify-rt.sh`)."

**Not** supported (do not say):

- ❌ "Hard real-time certified" / "WCET-bounded" / "deterministic latency."
- ❌ "Allocation-free" or "real-time allocation" on the publish/subscribe paths.
- ❌ "Formally verified DDS" / any claim covering RustDDS.
- ❌ "Panic-free codec" — the runtime codec can still panic on malformed input
  (OOM `assert` in `alloc_buf`); only the extracted arithmetic cores are
  panic-free.

A fair one-liner: *"roscmp's real-time arithmetic cores — CDR alignment, layout
rounding, timer scheduling, and tunnel backpressure — carry machine-checked
(Creusot/Why3) proofs of panic-freedom, correct alignment/bounds, and queue
boundedness; timing, allocation latency, and RustDDS remain out of scope."*
