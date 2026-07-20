# Real-Time Properties Of Roswell Hot Paths

This document reports two things, honestly separated:

1. **A hot-path audit** (Part A): where the publish/subscribe/spin/tunnel paths
   allocate, loop, recurse, panic, block, and where Roswell's code ends and
   RustDDS's begins.
2. **Machine-checked proofs** (Part B): the pure arithmetic cores of those
   paths carry Creusot contracts discharged by Why3/SMT. The claims here are
   exactly as strong as the proofs, and no stronger.

Reproduce the proofs with `scripts/verify-rt.sh` (requirements documented at the
top of that script). The normal build and test gate (`scripts/check.sh`) is
untouched by any of this: Creusot is opt-in behind the `verify` feature and the
`--cfg creusot` flag, and `creusot-std` is never compiled into a normal build.

## Part A: Hot-Path Audit

Scope: Roswell's own code. RustDDS is out of scope and unaudited; the
boundary is called out for each path. "Bounded" below means bounded by message
size / configured capacity, not constant-time.

### 1. Publish Path: `dynamic::encode` To Raw Publish

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
  definition, not by input. Recursion is bounded.
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
  `self.writer.write(...)` (`roswell-ros2-compat/src/raw.rs:668`). Everything past that
  call, including serialization copies, history cache, RTPS, sockets, internal
  locks, and blocking under `max_blocking_time`, is RustDDS and out of scope.

### 2. Subscribe Path: Take To `dynamic::decode`

`RawDdsSubscriber::take` (`roswell-ros2-compat/src/raw.rs:732`) pulls a sample from
RustDDS, then `DynamicType::decode` (`src/dynamic.rs:324`) writes it into caller
memory.

- **Allocations.** `decode_message` borrows its layout, so there is no
  per-message layout allocation. `alloc_buf` allocates per sequence
  (`src/dynamic.rs:348`), and `store_ros_string` allocates per string
  (`src/dynamic.rs:642`). Both are bounded by message content.
- **Sequence bounds.** `Reader::read_seq_len` (`src/cdr_runtime.rs`) rejects any element count
  exceeding the bytes remaining in the buffer (every element occupies ≥ 1 wire
  byte — even an empty nested message encodes a dummy octet), returning
  `Truncated` before any allocation. The guard covers the dynamic codec, the
  hand-rolled message modules, and the generated runtime alike (the generated
  preamble embeds `cdr_runtime.rs` verbatim; `msgs.rs` regenerated). A
  malformed prefix is now bounded to a `remaining-bytes`-sized allocation at
  worst.
- **Recursion / RustDDS boundary.** Same nesting bound as encode; the boundary
  is `reader.take_next_sample()` (`roswell-ros2-compat/src/raw.rs:734`).

### 3. `node::spin_once` Tick

`Node::spin_once` (`roswell-ros2-compat/src/node.rs:330`): `loop { drain(); … sleep }`.

- **Blocking.** `std::thread::sleep(...)` (`roswell-ros2-compat/src/node.rs:338`) — the
  executor blocks up to the caller's timeout by design.
- **`drain`** (`node.rs:308`) iterates existing timer/subscription/service Vecs;
  no per-tick allocation of its own (downstream `sub.poll()`/`service.serve()`
  decode and thus allocate).
- **Timer catch-up.** `Timer::fire` (`node.rs`) advances the deadline via the
  verified `roswell_verify::next_fire_after` function (see Part B). The clock fallbacks
  now saturate to `i64::MAX - 1`, so the proof's `now < i64::MAX` precondition
  holds at every call site.
- **Locks.** Shutdown is an `AtomicBool` (`node.rs:359,364`), no mutex.

### 4. Tunnel Queue Policy

- **Bounded queue.** `OutboundQueue::enqueue` (`roswell-ros2-compat/src/tunnel.rs:677`)
  admits frames through the verified `roswell_verify::plan_enqueue` drop policy
  (see Part B); `pop_next` is O(1) across three priority `VecDeque`s. Backlog is
  bounded by `max_pending` per channel.
- **Duplicate window.** `TopicBridgeRx::received_sequences` is a
  `BTreeSet<u64>` sliding window capped at `DEDUP_WINDOW = 4096` entries
  (oldest sequence evicted). Memory is bounded for indefinite uptime; a
  duplicate arriving more than a window late would re-publish, which the
  near-term retry design makes unlikely.
- **Locks.** `TunnelReliabilityHandle` is `Arc<Mutex<…>>`; the TX/RX paths take
  `.lock().expect("… poisoned")` (`tunnel.rs:319…`). Uncontended fast, but a
  poisoned lock **panics**, and a contended lock blocks.
- **Frame I/O DoS bound.** Inbound frames are length-prefixed and rejected above
  `MAX_FRAME_LEN = 64 MiB` (`tunnel.rs:18`) before `vec![0; len]` allocates, so a
  hostile length field cannot request an unbounded buffer.
- **Time.** `now_system_nanos` saturates (`i64::try_from(...).unwrap_or(i64::MAX)`),
  no panic.

## Part B: Machine-Checked Properties

The pure arithmetic cores of the paths above live in the dependency-free
**`roswell-verify`** crate and are the *production* implementations the hot paths
call; they are not parallel copies. They are isolated there because Creusot cannot
translate the surrounding crates (recursive IR enums in `ast`/`idl`, `unsafe`
pointer code in `dynamic`, RustDDS, nom); the small pure crate translates in
full. Contracts are erased in normal builds.

Verified with cargo-creusot 0.13.0-dev, toolchain nightly-2026-06-22, Why3
1.8.2, why3find 1.3.0, provers alt-ergo 2.6.2 / z3 4.15.3 / cvc5 1.3.1 / cvc4
1.8. Each is discharged with four files proved, and the proof sessions are
checked in.

### Proven properties

| Function (`roswell-verify`) | Production caller | Property proven | Artifact |
|---|---|---|---|
| `pad_to(off, a)` | CDR `Writer`/`Reader` alignment (`src/cdr_runtime.rs`, pinned by equivalence test) | `a>0` ⇒ no panic (no div-by-zero, no underflow in `a-off%a`); `result < a`; `(off+result) % a == 0` (correct alignment) | `verif/roswell_verify_rlib/pad_to/proof.json` |
| `round_up(off, align)` | `dynamic::round_up` layout offset rounding (`src/dynamic.rs:594`) | `align>0` ∧ `off+align ≤ usize::MAX` ⇒ no overflow; `off ≤ result < off+align`; `result % align == 0` | `verif/roswell_verify_rlib/round_up/proof.json` |
| `next_fire_after(next_fire, period, now)` | `Timer::fire` catch-up (`roswell-ros2-compat/src/node.rs:733`) | `period≥1` ∧ `now<i64::MAX` ⇒ **loop terminates** (variant `now-cur`); `result ≥ next_fire` (deadline never rewinds); `result > now` | `verif/roswell_verify_rlib/next_fire_after/proof.json` |
| `plan_enqueue(len, max_pending, drop_oldest)` | `OutboundQueue::enqueue` drop policy (`roswell-ros2-compat/src/tunnel.rs:693`) | `max_pending≥1` ⇒ resulting length never rises above `max_pending` unless already above it; a *net* enqueue happens only with strict room; never evicts from an empty lane | `verif/roswell_verify_rlib/plan_enqueue/proof.json` |

"No panic" = Creusot's automatic safety VCs (integer overflow/underflow,
division by zero, indexing) are discharged in addition to the functional
contract.

The CDR `pad_to` is embedded verbatim into generated bindings, so it cannot call
an external crate; `cdr::tests::pad_to_matches_verified_core` (`src/cdr.rs`)
pins the embedded formula to the verified `roswell_verify::pad_to` over every CDR
alignment and a wide offset range, transferring the proof to that copy.

### A bug the proof surfaced

`next_fire_after` terminates **only** for `now < i64::MAX`. The clock fallback
`tick_nanos → unwrap_or(i64::MAX)` (`node.rs`) could, in the ~292-year-past-epoch
/ clock-failure corner, feed `now == i64::MAX`, at which the original
`while next_fire_nanos <= now` loop would spin forever. The precondition named
that corner precisely; both fallbacks now saturate to `i64::MAX - 1`, closing
it at every call site.

## What Is Not Proven

These proofs are about **arithmetic and control-flow correctness**, not timing.
None of the following is established here:

- **No WCET or timing bound.** Creusot proves termination, not duration.
  There is no worst-case execution time, no cycle count, no deadline guarantee.
- **Allocator behavior.** The publish/subscribe paths call the global allocator
  (`Vec` growth, sequence buffers, string copies). Allocation *latency* and
  fragmentation are unmodeled; a real RT deployment would need a bounded /
  real-time allocator and pre-sizing, neither proven here.
- **The full encode/decode walk is not proven.** Only the arithmetic cores are.
  The `unsafe` pointer walk, UTF-8 assumptions, and malformed-input handling are
  audited and tested, not verified.
- **RustDDS internals.** Serialization, history caches, RTPS, sockets, locks,
  and blocking under QoS are entirely out of scope and unverified.
- **Operating-system behavior.** Scheduling, priority inversion, interrupt
  latency, and `thread::sleep` accuracy are unmodeled.
- **Concurrency.** The `Arc<Mutex<TunnelReliabilityState>>` path is audited, not
  verified; no lock-freedom or deadlock-freedom is claimed.

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

- "Hard real-time certified," "WCET-bounded," or "deterministic latency."
- "Allocation-free" or "real-time allocation" on the publish/subscribe paths.
- "Formally verified DDS" or any claim covering RustDDS.
- "Fully verified codec." The full runtime codec is tested and audited, but
  only the extracted arithmetic cores are machine-checked.

A fair summary: "Roswell's real-time arithmetic cores for CDR alignment, layout
rounding, timer scheduling, and tunnel backpressure carry machine-checked
(Creusot/Why3) proofs of panic-freedom, correct alignment/bounds, and queue
boundedness; timing, allocation latency, and RustDDS remain out of scope."
