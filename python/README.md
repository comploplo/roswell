# roswell (Python)

`pip install`-able Python that speaks ROS 2 with **zero ROS installation**: no
`rosidl` codegen, no PyO3 build step, and no callback-group ceremony.

The package is a small asyncio-native client over Roswell's Rust runtime.
Roswell keeps the ROS message semantics — parsing, layout, CDR serialization,
QoS, transport, and reply correlation — in one shared implementation, then
exposes a pleasant Python surface through `ctypes`.

## Install

```bash
pip install roswell            # self-contained wheel: bundles the compiled
                              # runtime + a set of common ROS interfaces
pip install roswell[numpy]     # + numpy for zero-copy array views
```

Published wheels are platform wheels (`py3-none-<platform>`) carrying the prebuilt
`roswell_c` shared library — no ROS, no `rosidl`, no Rust toolchain, no compiler
needed to *use* it. Common interface definitions (`std_msgs`, `geometry_msgs`,
`sensor_msgs`, `example_interfaces`, …) ship inside the wheel, so
`load_type("geometry_msgs/msg/Twist")` works with no file paths.

### Raspberry Pi / ARM Linux

Roswell targets embedded nodes: the runtime is a compact cdylib (idle `Node`
resident memory is a few MB over the interpreter) with a fast import, and the
package is pure `ctypes` — **numpy is optional**. Install the manylinux
`aarch64` wheel on Pi-class ARM Linux and skip the numpy extra to keep the
footprint minimal; primitive arrays then come back as plain Python lists instead
of zero-copy numpy views:

```bash
pip install roswell            # no numpy — lists for array fields
pip install roswell[numpy]     # + numpy for zero-copy views (if you want them)
```

## Quickstart

```python
import asyncio, roswell

async def main():
    node = roswell.Node("listener", domain=0)
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
node = roswell.Node("talker", domain=0)
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

## Everyday Use

- **asyncio-native**: `async for msg in sub`, `await client.call(req)`. One
  background thread per node multiplexes all readers (`ros_wait`) and dispatches
  into your event loop — never a thread per subscription, never a busy-poll.
- **Sync too**: `sub.take()`, `for msg in sub.messages(timeout=...)`,
  `client.call_sync(req)`.
- **numpy zero-copy**: primitive arrays/sequences (e.g. `float64[]`, `uint8[]`)
  are exposed as `numpy` views over the underlying buffer when numpy is present
  (valid until the owning message is finalized), and as plain lists otherwise.
- **Clear QoS warnings**: incompatible QoS with a peer surfaces automatically as
  a `roswell.QosIncompatibleWarning`.
- **Messages are runtime-typed**: fields are plain attributes
  (`msg.header.frame_id = "map"`, `msg.data = np.zeros(...)`).

All parsing, layout, CDR (de)serialization, QoS, transport, and reply
correlation live in Rust; this package is ctypes bindings + asyncio plumbing.

## Parameters

Declare, read, and update node parameters. The first parameter call stands up a
parameter server (a background thread in the Rust runtime) so `ros2 param
get/set/list` sees the node and every change publishes `/parameter_events`:

```python
node = roswell.Node("driver", domain=0)
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
    node = roswell.Node("ticker", domain=0)
    pub = node.publisher("/chatter", node.load_type("std_msgs/msg/String"))

    def tick():
        m = pub.new(); m.data = "hi"; pub.publish(m)

    node.create_timer(0.1, tick)            # 10 Hz
    await asyncio.Event().wait()            # spin

asyncio.run(main())
```

## Develop

```bash
cargo build -p roswell-c             # builds the shared library into target/
pip install -e "python[test]"        # editable install; numpy + pytest
python -m pytest python/tests -v
# or, without activating a venv:
uv run --project python --extra test pytest python/tests
python python/tests/bench_pointcloud.py
```

At import time, the shared library is located via `ROSWELL_LIB` (if set), then the bundled
`roswell/_lib/` (wheels), then the dev `target/{release,debug}` tree. Bundled
interfaces are found under `roswell/interfaces/` (wheels) or `samples/` (checkout),
overridable with `ROSWELL_INTERFACES`.

## Build release wheels

```bash
uv run --project python --extra release cibuildwheel --output-dir python/dist python
```

That one command builds the configured wheel set: Linux x86_64, Linux aarch64,
macOS universal2, and Windows AMD64. Each wheel bundles the compiled Rust
runtime plus common interfaces and is tagged `py3-none-<platform>`, so it is not
tied to one CPython ABI. Build the sdist with `python -m build --sdist python/`
and check everything with `twine check python/dist/*`.

A Rust toolchain is required to *build* from source, but not to install or use a
published wheel. The package version is single-sourced from the crate
(`roswell-c/Cargo.toml`).

## QoS

```python
from roswell import QosProfile
pub = node.publisher("/scan", T, qos=QosProfile.preset("sensor_data"))
sub = node.subscribe("/scan", T, qos=QosProfile(reliability="best_effort", depth=5))
```

## Visualize With Foxglove

Roswell has no in-process visualization yet, but it interoperates with
[Foxglove Studio](https://foxglove.dev/) through MCAP files. Record any topics
your Python nodes publish with the `bag_record` binary from the Rust workspace,
then open the resulting `.mcap` in Foxglove.

Record (type-blind — no ROS install needed), from the repo root:

```bash
# Named topics with their ROS types (matches the domain your Node uses):
cargo run --release -p roswell-ros2-compat --bin bag_record -- \
    --output chatter.mcap --domain 0 --topic /chatter:std_msgs/msg/String

# ...or discover and record everything on the graph for 10 seconds:
cargo run --release -p roswell-ros2-compat --bin bag_record -- \
    --output session.mcap --domain 0 --all --duration 10
```

Recording stops on `--duration`, on Enter/Ctrl-D at a terminal, or on Ctrl-C
(already-flushed chunks stay readable). A per-topic message-count summary prints
on a clean stop.

Then in Foxglove Studio choose **Open local file…** and select the `.mcap` — the
schemas travel inside the file, so panels (Raw Messages, Plot, 3D, …) work with
no extra setup. To sanity-check a recording without Foxglove, replay it:

```bash
cargo run --release -p roswell-ros2-compat --bin bag_play -- session.mcap --domain 0
```
