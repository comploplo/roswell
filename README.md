# Roswell

Roswell exists because understanding ROS message semantics should not require
adopting the entire ROS build system and runtime.

Roswell is a lightweight implementation of the useful parts of ROS
communication: interface parsing, message layout, CDR serialization, generated
bindings, and practical interoperability with ordinary ROS 2 nodes. It parses
standard ROS interface files (`.msg`, `.srv`, `.action`, and a useful subset of
IDL), centralizes their semantics in one layout model, and uses that model from
Rust, C, Python, and embedded-friendly `no_std` Rust.

The easiest way to use it is `pip install roswell`, which installs a small
Python client that can publish, subscribe, call services, serve services, and
work with common ROS interfaces without a ROS installation.

## Why Roswell

ROS 2 is powerful, but its language bindings and generated type support can be a
lot to carry into small tools, Python services, and embedded-adjacent systems.
Roswell separates the message model, serialization, and interoperability from
the rest of the ROS development environment.

Roswell is useful when you want to:

- write Python tools that join a ROS 2 graph without installing ROS
- build small services, scripts, and data pipelines around ROS messages
- generate Rust, C, Python, or `no_std` Rust bindings from ROS interfaces
- record and replay ROS 2 CDR samples as MCAP
- bridge selected topics between embedded firmware and a ROS 2 graph
- inspect or test ROS wire compatibility from a compact codebase

The design center is one canonical implementation of ROS message semantics:

- one parser and layout model shared by Rust, C, and Python
- one CDR serializer, tested against `ros:jazzy`
- runtime type loading for `.msg` and `.srv` files
- a Python package built on `ctypes`, with no PyO3 or ROS install required
- ROS 2 compatibility through `roswell-ros2-compat`
- no_std message generation for microcontroller firmware

The goal is not to replace ROS 2. The goal is to make it easier to build small,
understandable programs that can still join a ROS 2 graph.

## Mental Model

```text
ROS interface definitions
        |
        v
Roswell parser + resolver + layout model
        |
        v
ROS-compatible CDR bytes
        |
        +--> generated Rust / C / Python / no_std Rust bindings
        |
        +--> C ABI used by the Python package
        |
        +--> RustDDS-backed ROS 2 compatibility runtime
        |
        +--> MCAP tools and embedded topic bridges
```

Every language binding, runtime surface, and embedded target is generated from
or routed through that same understanding of ROS message layouts.

## Try It

Generate bindings from ROS interface files:

```sh
roswell --lang all --out generated samples/geometry_msgs/msg/*.msg
roswell --lang rust samples/geometry_msgs/msg/Point.msg
roswell --lang rust --dds samples/std_msgs/msg/String.msg
```

Use the Python runtime:

```python
import asyncio
import roswell

async def main():
    node = roswell.Node("listener", domain=0)
    Twist = node.load_type("geometry_msgs/msg/Twist")

    async with node.subscribe("/cmd_vel", Twist) as sub:
        async for msg in sub:
            print(msg.linear.x)

asyncio.run(main())
```

Drive a ROS 2 simulator:

```sh
cargo run -p roswell-ros2-compat --bin turtle_teleop
```

Record topics to MCAP:

```sh
cargo run --release -p roswell-ros2-compat --bin bag_record -- \
  --output session.mcap --domain 0 --all --duration 10
```

## What Works Today

- `.msg`, `.srv`, `.action`, and IDL parsing
- Rust, C, Python, and no_std Rust code generation
- XCDR1 encode/decode, with XCDR2 coverage in the generated/no_std paths
- runtime type loading for Python and C
- ROS 2 pub/sub, services, actions, parameters, `/clock`, `/tf`, `/tf_static`,
  `/rosout`, diagnostics, and type-description services
- MCAP record/replay of ROS 2 CDR samples
- TCP and USB topic bridges for selected topics

Interop is tested against a `ros:jazzy` container. See
the [`roswell-ros2-compat` documentation](roswell-ros2-compat/) for the ROS 2
compatibility layer and the [Python package documentation](python/).

## Project Layout

- `src/` — parser, resolver, layout IR, CDR runtime, and code generators
- `roswell-ros2-compat/` — ROS 2 compatibility runtime built on RustDDS, plus
  tools and bridge binaries
- `roswell-c/` — handle-based C ABI used by the Python package
- `python/` — installable Python client
- `roswell-build/` — `build.rs` integration for Rust projects
- `roswell-tunnel-core/` — no_std tunnel frame codec for embedded links
- `hil-renode/` — Renode-based hardware-in-loop style firmware tests
- `docs/RT.md` — real-time audit and machine-checked arithmetic proofs

## Verification

Roswell's primary correctness concern is wire compatibility: generated layouts
and serialized bytes should match standard ROS 2 implementations. Verification
therefore focuses on the parser/layout/CDR core first, then the runtime surfaces
that depend on it.

- Cross-language layout tests compile generated Rust, C, and Python and compare
  `sizeof` plus field offsets.
- CDR tests compare Roswell bytes against `ros:jazzy`.
- ROS 2 compatibility tests exercise pub/sub, services, actions, QoS events,
  graph discovery, bags, sim time, and turtlesim.
- `roswell-verify` contains small production arithmetic cores with Creusot/Why3
  proofs for alignment, layout rounding, timer catch-up, and tunnel queue bounds.

Roswell is intentionally direct about what is proven, what is tested, and what
still depends on the underlying DDS transport.

Run the local quality gate:

```sh
./scripts/check.sh
```

For the proof-specific workflow, see [`docs/RT.md`](docs/RT.md).

## Status

Roswell is pre-1.0. The core compiler/runtime is usable, but APIs may still
change while the project settles.

Known limits:

- ROS 2 compatibility currently rides RustDDS, so RustDDS behavior is part of
  the transport boundary.
- Large same-host data is currently limited by the RustDDS transport path; see
  the [benchmark notes](python/benchmarks/) for current numbers.
- The MCAP tools are MCAP-native, not full rosbag2 parity.
- The no_std embedded path speaks selected topics through a bridge; it is not a
  micro-ROS replacement.

Roswell separates the useful parts of ROS communication: the message model,
serialization, and interoperability. It provides them without the rest of the
ROS development environment. It lets ordinary programs participate in a ROS
ecosystem without requiring a ROS workspace, while staying compatible with
existing ROS 2 systems where practical.

## Frequently Asked Questions

### Why would I use this instead of rclpy?

Use `rclpy` when you want the complete, distribution-supported ROS 2 Python
experience and are comfortable installing ROS. Use Roswell when you want a
small Python program, service, data tool, or deployment that can communicate
with a ROS 2 graph without carrying a ROS installation. Roswell deliberately
implements less: it is not a drop-in replacement for every `rclpy`, `rcl`, or
executor feature.

### Which ROS distributions and RMW implementations work?

Wire compatibility is tested against ROS 2 Jazzy. Roswell participates through
standard RTPS/DDS rather than linking to a particular ROS distribution or RMW,
so distribution labels do not form an API boundary. Peers using other ROS 2
distributions or RMW implementations may interoperate, but combinations not in
the interop suite should be treated as unverified. Compatibility reports are
welcome, especially for Cyclone DDS and Fast DDS peers.

### Does Roswell replace DDS or implement an RMW?

No. `roswell-ros2-compat` uses RustDDS for discovery and transport. Roswell owns
the ROS interface model, layouts, CDR payloads, bindings, and ROS-facing runtime
behavior around that transport. RustDDS behavior remains part of the
compatibility and performance boundary.

### Can I use custom messages?

Yes. The compiler accepts `.msg`, `.srv`, `.action`, and the supported IDL
subset. Python can load interface files at runtime, and Rust projects can
generate bindings from `build.rs` with `roswell-build`. The wheel includes
common interfaces for use without additional paths; custom packages do not
have to be submitted to Roswell.

### Does it support ROS 1 or Noetic?

The parser understands ROS-style message definitions, including ROS 1 `time`
and `duration` syntax, but the runtime speaks ROS 2 RTPS/DDS. Roswell does not
currently join a ROS 1 graph. A ROS 1 bridge remains the appropriate boundary
for Noetic systems. The Python package supports Python 3.10 and newer, so it
does not cover Noetic's common Python 3.8 environment today.

### Can embedded firmware speak ROS through Roswell?

Yes, with a deliberately narrow model. Roswell generates bounded,
allocation-free `no_std` message code. `roswell-tunnel-core` carries selected
CDR topics over UART, USB CDC, or another byte stream to a host bridge. The MCU
does not run DDS, discovery, an executor, or the full ROS graph protocol. See
the [embedded example](embedded/) for the working path.

### Is this a micro-ROS replacement?

No. micro-ROS brings the ROS 2 client model to microcontrollers through the
XRCE-DDS ecosystem. Roswell's embedded path is a smaller selected-topic bridge.
Choose micro-ROS when you need its established client, agent, tooling, and
ecosystem; choose Roswell's tunnel when bounded generated messages and a narrow
host-mediated link are the better fit.

### How is the Python package built?

Python uses `ctypes` over the same C ABI exported by `roswell-c`. Published
wheels bundle the compiled Rust runtime and common interface definitions, so
users do not need Rust or ROS installed. The release set targets Linux x86_64,
Linux aarch64, macOS universal2, and Windows AMD64. Because the extension uses a
C ABI rather than CPython's extension ABI, each platform wheel is usable across
the supported Python versions.

### What are the performance limits?

Message parsing and CDR serialization are native Rust, and NumPy arrays can use
zero-copy views in Python. End-to-end throughput and latency still depend on
RustDDS and the host network stack. In particular, Roswell does not currently
provide a shared-memory transport for large same-host samples. Benchmark the
actual message sizes, QoS, peers, and network used by your robot before making a
deployment decision.

### Does it support ROS security and every QoS policy?

Not yet. Common reliable, best-effort, transient-local, history, and resource
limit behavior is present, including incompatible-QoS events. SROS2/DDS Security
is not currently supported, and Roswell should not be placed on an untrusted DDS
network on the assumption that it supplies authentication or encryption.

### Is Roswell production-ready?

Roswell is an alpha, pre-1.0 project. Its compiler, serialization core, Python
surface, and compatibility runtime have substantial automated coverage, but the
API may change and the tested platform/RMW matrix is still intentionally small.
It is ready for tools, experiments, evaluation, and carefully scoped
deployments. Production users should pin versions and validate against their
own graph.

### How does Roswell avoid duplicating its APIs and wire logic?

The parser, resolver, layout model, and CDR implementation are the source of
truth. Generated bindings and runtime-loaded types use that model; Python calls
the Rust runtime through the `roswell-c` ABI rather than maintaining another
serializer or DDS client. The public language APIs are idiomatic wrappers over
the same message and transport semantics, not independent implementations.

## Contributing

Issues, experiments, and careful bug reports are welcome. The most useful
reports include the ROS distribution, platform, message type, QoS, and whether
the peer was `rclpy`, `rclcpp`, or another Roswell node.

Before sending a change, run:

```sh
./scripts/check.sh
```

Roswell is dual-licensed under [Apache 2.0](LICENSE-APACHE) or
[MIT](LICENSE-MIT), at your option.
