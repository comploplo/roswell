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

### Raspberry Pi / ARM Linux

roscmp targets embedded nodes: the runtime is a compact cdylib (idle `Node`
resident memory is a few MB over the interpreter) with a fast import, and the
package is pure `ctypes` — **numpy is optional**. Install the manylinux
`aarch64` wheel on Pi-class ARM Linux and skip the numpy extra to keep the
footprint minimal; primitive arrays then come back as plain Python lists instead
of zero-copy numpy views:

```bash
pip install roscmp            # no numpy — lists for array fields
pip install roscmp[numpy]     # + numpy for zero-copy views (if you want them)
```

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

## Parameters

Declare, read, and update node parameters. The first parameter call stands up a
parameter server (a background thread in the Rust runtime) so `ros2 param
get/set/list` sees the node and every change publishes `/parameter_events`:

```python
node = roscmp.Node("driver", domain=0)
node.declare_parameter("speed", 1.5)        # float
node.declare_parameter("gain", 7)           # int
node.declare_parameter("frame", "base_link")# str
node.declare_parameter("enabled", True)     # bool

node.get_parameter("speed")                 # -> 1.5
node.set_parameter("gain", 9)               # publishes a /parameter_events update
node.list_parameters()                      # -> ["enabled", "frame", "gain", "speed"]
```

Scalar types (`bool`, `int`, `float`, `str`) are supported — the common case for
node configuration. Values live in Rust; the Python surface is just the typed
call across the FFI.

## Timers

`create_timer` runs a periodic callback (sync or `async`) on the asyncio loop —
the idiomatic "publish at N Hz" node loop. Timing is plumbing, so it lives in
Python; cancel it explicitly or let `node.close()` stop it:

```python
async def main():
    node = roscmp.Node("ticker", domain=0)
    pub = node.publisher("/chatter", node.load_type("std_msgs/msg/String"))

    def tick():
        m = pub.new(); m.data = "hi"; pub.publish(m)

    node.create_timer(0.1, tick)            # 10 Hz
    await asyncio.Event().wait()            # spin

asyncio.run(main())
```

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

## Visualize with Foxglove

roscmp has no in-process visualization, but it interoperates with
[Foxglove Studio](https://foxglove.dev/) through MCAP files. Record any topics
your Python nodes publish with the `bag_record` binary from the Rust workspace,
then open the resulting `.mcap` in Foxglove.

Record (type-blind — no ROS install needed), from the repo root:

```bash
# Named topics with their ROS types (matches the domain your Node uses):
cargo run --release -p roscmp-dds --bin bag_record -- \
    --output chatter.mcap --domain 0 --topic /chatter:std_msgs/msg/String

# ...or discover and record everything on the graph for 10 seconds:
cargo run --release -p roscmp-dds --bin bag_record -- \
    --output session.mcap --domain 0 --all --duration 10
```

Recording stops on `--duration`, on Enter/Ctrl-D at a terminal, or on Ctrl-C
(already-flushed chunks stay readable). A per-topic message-count summary prints
on a clean stop.

Then in Foxglove Studio choose **Open local file…** and select the `.mcap` — the
schemas travel inside the file, so panels (Raw Messages, Plot, 3D, …) work with
no extra setup. To sanity-check a recording without Foxglove, replay it:

```bash
cargo run --release -p roscmp-dds --bin bag_play -- session.mcap --domain 0
```
