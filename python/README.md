# roscmp (Python)

`pip install`-able Python that speaks ROS2 (RTPS/DDS) with **zero ROS
installation** — no `rosidl` codegen, no PyO3, no executor/callback-group
ceremony. A thin, asyncio-native client over the roscmp Rust runtime.

## Install

```bash
pip install roscmp            # self-contained wheel: bundles the compiled
                              # runtime + a set of common ROS interfaces
pip install roscmp[numpy]     # + numpy for zero-copy array views
```

The wheel is a platform wheel (`py3-none-<platform>`) carrying the prebuilt
`roscmp_c` shared library — no ROS, no `rosidl`, no Rust toolchain, no compiler
needed to *use* it. Common interface definitions (`std_msgs`, `geometry_msgs`,
`sensor_msgs`, `example_interfaces`, …) ship inside the wheel, so
`load_type("geometry_msgs/msg/Twist")` works with no file paths.

## Quickstart

```python
import asyncio, roscmp

async def main():
    node = roscmp.Node("listener", domain=0)
    Twist = node.load_type("geometry_msgs/msg/Twist")   # bundled — no path, no deps
    async with node.subscribe("/cmd_vel", Twist) as sub:
        async for msg in sub:
            print(msg.linear.x)

asyncio.run(main())
```

`load_type` also takes an explicit `.msg` path plus `deps=[...]` for your own
interface files:

```python
T = node.load_type("my_pkg/msg/Custom.msg", deps=["my_pkg/msg/Helper.msg"])
```

Publish, call a service, and serve one:

```python
node = roscmp.Node("talker", domain=0)
Str = node.load_type("std_msgs/msg/String")
pub = node.publisher("/chatter", Str)
msg = pub.new(); msg.data = "hello"; pub.publish(msg)

req_t, resp_t = node.load_service("example_interfaces/srv/AddTwoInts")
client = node.client("/add_two_ints", req_t, resp_t)
req = client.new_request(); req.a, req.b = 41, 1
reply = await client.call(req, timeout=5.0)      # -> reply.sum == 42

def handler(request):
    resp = resp_t.alloc(); resp.sum = request.a + request.b; return resp
node.serve("/add_two_ints", (req_t, resp_t), handler)   # sync or async handler
```

## Ergonomics

- **asyncio-native**: `async for msg in sub`, `await client.call(req)`. One
  background thread per node multiplexes all readers (`rcm_wait`) and dispatches
  into your event loop — never a thread per subscription, never a busy-poll.
- **Sync too**: `sub.take()`, `for msg in sub.messages(timeout=...)`,
  `client.call_sync(req)`.
- **numpy zero-copy**: primitive arrays/sequences (e.g. `float64[]`, `uint8[]`)
  are exposed as `numpy` views over the underlying buffer when numpy is present
  (valid until the owning message is finalized), and as plain lists otherwise.
- **Loud QoS warnings**: incompatible QoS with a peer surfaces automatically as
  a `roscmp.QosIncompatibleWarning`.
- **Messages are runtime-typed**: fields are plain attributes
  (`msg.header.frame_id = "map"`, `msg.data = np.zeros(...)`).

All parsing, layout, CDR (de)serialization, QoS, transport, and reply
correlation live in Rust; this package is ctypes bindings + asyncio plumbing.

## Develop

```bash
cargo build -p roscmp-c            # builds the shared library into target/
pip install -e python/[test]       # editable install; numpy + pytest
pytest python/tests -v
python python/tests/bench_pointcloud.py
```

The shared library is located via `ROSCMP_LIB` (if set), then the bundled
`roscmp/_lib/` (wheels), then the dev `target/{release,debug}` tree. Bundled
interfaces are found under `roscmp/interfaces/` (wheels) or `samples/` (checkout),
overridable with `ROSCMP_INTERFACES`.

## Build the wheel

```bash
python -m build --wheel python/    # runs `cargo build -p roscmp-c --release`,
                                   # bundles the cdylib + interfaces, tags the
                                   # wheel py3-none-<platform>
```

A Rust toolchain is required to *build* the wheel, but not to install or use one.
The wheel version is single-sourced from the crate (`roscmp-c/Cargo.toml`).

## QoS

```python
from roscmp import QosProfile
pub = node.publisher("/scan", T, qos=QosProfile.preset("sensor_data"))
sub = node.subscribe("/scan", T, qos=QosProfile(reliability="best_effort", depth=5))
```
