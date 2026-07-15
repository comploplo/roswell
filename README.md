# roscmp

A from-scratch compiler for ROS message definitions. It parses `.msg` files and
emits **FFI-compatible** type bindings for Rust, C, and Python that all share a
single memory layout — so a struct produced for one language has byte-identical
size and field offsets in the others.

## Why

ROS ships a separate code generator per language, each reimplementing layout and
serialization. `roscmp` keeps **one source of truth for the C ABI** and projects
it into every language, with a parser built from first principles on `nom`.

## Pipeline

```
.msg  →  parser (nom)  →  AST  →  resolver  →  layout-aware IR  →  codegen  →  Rust / C / Python
```

- **parser** (`src/parser.rs`) — line-oriented ROS2 `.msg`: primitives, `byte`/`char`,
  bounded `string<=N`, namespaced types (`pkg/Type`, `pkg/msg/Type`), arrays
  (`[]`, `[N]`, `[<=N]`), constants, and field defaults. Quote-aware comment
  stripping; errors carry 1-based line numbers.
- **resolver** (`src/resolve.rs`) — normalizes type references (bare `Header` →
  `std_msgs/Header`), injects builtins (`builtin_interfaces/Time`, `Duration`,
  `std_msgs/Header`), validates, and topologically orders messages.
- **IR** (`src/ir.rs`) — every primitive knows its (size, align) and its spelling
  in each target.
- **codegen** (`src/codegen/`) — `#[repr(C)]` Rust, a C99 header, and
  `ctypes.Structure` Python. Strings/sequences use a shared
  `{ data, size, capacity }` triple; fixed arrays are inline; nested messages
  embed by value.

## ABI model

| `.msg`            | layout                                  |
|-------------------|-----------------------------------------|
| scalar primitive  | fixed-width int/float                   |
| `string`          | `{ char* data; size_t size, capacity }` |
| `T[]`, `T[<=N]`   | `{ void* data; size_t size, capacity }` |
| `T[N]`            | inline array                            |
| nested message    | embedded by value (pointer in sequences)|

## C & Python bindings (`roscmp-c/`, `python/`)

`roscmp-c` exposes the whole runtime — type loading, pub/sub, services, QoS,
graph, node identity — as a handle-based C ABI (`include/roscmp.h`), driven by
a runtime layout codec (`src/dynamic.rs`): any `.msg`/`.srv` works with no
codegen step, byte-identical to the generated serializers. `python/` is a thin
ctypes client over it — asyncio-native (`async for msg in sub`,
`await client.call()`), zero-copy numpy views for arrays, loud QoS-mismatch
warnings, zero ROS installation. Publishing a ~10 MB pointcloud from Python
takes ~25 ms (rclpy's cited figure for the same shape: ~92 ms).

```python
node = roscmp.Node("listener")
Twist = node.load_type("geometry_msgs/msg/Twist.msg", deps=[...])
async with node.subscribe("/cmd_vel", Twist) as sub:
    async for msg in sub:
        print(msg.linear.x)
```

## Usage

```sh
roscmp --lang all --out generated samples/geometry_msgs/msg/*.msg
# or a single language to stdout:
roscmp --lang rust samples/geometry_msgs/msg/Point.msg
# Rust + RTPS registration glue for roscmp-dds (emits `CdrMsg` impls):
roscmp --lang rust --dds samples/std_msgs/msg/String.msg
```

Package/name are inferred from the `<package>/msg/<Name>.msg` path.

## Verification

`tests/layout_tests.rs` is the core guarantee: it generates all three backends,
compiles/runs each (`rustc`, `cc`, `python3`) to report `sizeof` and per-field
offsets, and asserts they agree — covering misaligned scalars, nested messages,
strings, fixed arrays, and sequences.

```sh
cargo test          # parser unit tests + cross-language layout tests
```

## Development / quality gate

One command runs everything pre-commit runs:

```sh
./scripts/check.sh        # rustfmt --check · clippy pedantic (-D warnings) · cargo-deny · tests
```

- **clippy** is `pedantic` with a curated allow-list in `[workspace.lints]`
  (intentional CDR casts, codegen string-building, etc.).
- **cargo-deny** (`deny.toml`) checks advisories, licenses, bans, sources.
- **pre-commit** (`.pre-commit-config.yaml`) adds typos + whitespace/EOF hooks on
  top of fmt/clippy/deny. Enable with `pre-commit install`. Refresh dependencies
  on demand with `pre-commit run cargo-update --hook-stage manual`.

## Roadmap

- [x] **M1** — ROS2 `.msg` parser + ABI-matched type codegen (Rust/C/Python)
- [x] **M2** — CDR (XCDR1) serialize/deserialize, **wire-verified against `ros:jazzy`**
  - [x] alignment-aware CDR runtime (encapsulation header, strings, sequences, arrays)
  - [x] generated Rust `to_cdr`/`from_cdr`/`fini`, verified by spec vectors + round-trip
  - [x] one serializer exposed to C/Python via FFI (all three emit identical bytes)
  - [x] live byte-diff vs a real `ros:jazzy` node — **byte-for-byte match**
- [x] **M3** — frontend breadth
  - [x] `.srv` / `.action` parsing; ROS1 `time`/`duration`; CLI dispatch by extension
  - [x] **RIHS01 type hashes** (`src/typehash.rs`) — byte-exact vs `ros:jazzy`, emitted as `<Msg>__TYPE_HASH` consts
  - [x] **OMG IDL frontend** (`src/idl.rs`) — modules/structs/sequences/scoped names/annotations → same IR as `.msg`
- [x] **M4** — transport behind a trait (RustDDS backend) — see `roscmp-dds/`
  - [x] **pub, sub, teleop, and a service all interop with vanilla ROS2** over real RTPS, using our CDR (`roscmp-dds/interop_test.sh`): `talker`→`ros2 topic echo`, `listener`←`ros2 topic pub`, `teleop` Twist on `/cmd_vel`, `add_server`←`ros2 service call` (→ `sum=7`, correlated via RTPS sample identity)
  - [x] `Transport` trait extracted (`roscmp-dds/src/transport.rs`); all bins ride it, ready for a second backend
  - [x] **per-topic QoS presets** (`Qos::{Default,SensorData,Latched}`) — interop-verified best-effort and transient_local (latched) endpoints
  - [x] **auto-generated DDS registration** — `--dds` emits a `CdrMsg` impl per message (no hand-written `impl_cdr!`); coverage broadened (incl. `sensor_msgs/Imu`)
  - [x] **reusable service client/server** (`roscmp-dds/src/service.rs`) — our `Client` calls a vanilla `demo_nodes_cpp` server (→ `sum=11`), correlated by RTPS sample identity
  - [x] **read-only graph introspection** (`graph` bin) — lists discovered topics/types, no `ros2` CLI needed
- [x] **M5** — drive a real ROS2 sim: `turtle_teleop` (terminal UI) drives **turtlesim** over RTPS — publishes `Twist` to `/turtle1/cmd_vel`, reads back `turtlesim/Pose` (a type our compiler generated). Headless proof: `roscmp-dds/turtle_sim_test.sh` (turtle moves, `MOVED=true`).

CDR design notes live in `src/cdr.rs`. The alignment-origin-reset assumption
after the encapsulation header is **confirmed** against ros:jazzy (see
`tests/cdr_tests.rs::jazzy_verified_wire_bytes`).
