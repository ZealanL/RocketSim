# rocketsim_vis

Use `rocketsim_vis` to connect your RocketSim simulation to the separate
`rocketsim-viewer` application. You can send simulation frames and debug
drawing. You can also receive driving controls and action requests.

This crate does not open a window or include a renderer. Your simulation
does not need the viewer's graphics dependencies. See the
[viewer guide](../rocketsim_viewer/README.md) if you only want to view or drive
an existing simulation.

## Try the workspace example

From a source checkout, [prepare the collision meshes](../README.md#prepare-the-collision-meshes),
then run:

```sh
cargo run -p rocketsim_vis --example minimal
```

This builds the matching viewer and shows a car driving for about five
seconds. To drive the car yourself, use the `drive` example instead:

```sh
cargo run --release --example drive
```

Select the first car with `1`. Close settings with `Escape` before driving.
Stop the drive example with Ctrl-C in its terminal.

## Add viewing to your application

Add matching simulation and connection crates to your `Cargo.toml`:

```toml
[dependencies]
rocketsim = "0.3.0"
rocketsim_vis = "0.3.0"
```

This guide uses the 0.3 API. Keep the crates on matching versions.

Before you run your application:

1. [Prepare the collision meshes](../README.md#prepare-the-collision-meshes)
   for the simulation. `init_from_default` reads `./collision_meshes/` relative
   to the process's working directory.
2. [Install or build the viewer](../rocketsim_viewer/README.md#install-the-viewer-separately).
   Adding `rocketsim_vis` to an external application does not install it.
3. Make `rocketsim-viewer` available on `PATH`, or set `ROCKETSIM_VIEWER` to
   its executable path.

### Send your first frames

This complete program loads the meshes, starts or connects to the default
viewer, and drives a car for about two seconds:

```rust,no_run
use std::{thread, time::Duration};

use rocketsim::{
    Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
};
use rocketsim_vis::ArenaVisExt;

fn main() -> std::io::Result<()> {
    init_from_default(true)?;

    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(None);
    arena.try_set_vis_enabled(true)?;
    arena.set_car_controls(car, CarControls::default().with_throttle(1.0));

    for _ in 0..240 {
        arena.step_tick();
        thread::sleep(Duration::from_micros(8_333));
    }

    arena.try_set_vis_enabled(false)?;
    Ok(())
}
```

Each `step_tick` advances the simulation by `1/120` second. The sleep makes
this small example run near real time. For interactive applications, use
[`TickPacer`](https://docs.rs/rocketsim_vis/latest/rocketsim_vis/pace/struct.TickPacer.html)
to pace the simulation and limit catch-up work. For batch simulation, omit
wall-clock sleeps and run the required number of ticks.

`try_set_vis_enabled(true)` uses `127.0.0.1:39201`. It starts a viewer if none
is listening there. A viewer started by the crate closes when its last
attachment or viewer handle is released. A viewer you start yourself stays
open after the simulation disconnects.

Use the `try_set_*` methods when your application needs to handle errors.
The methods without `try_` report errors to standard error instead.

## Connect to a viewer you start yourself

Start an installed viewer on a separate port:

```sh
rocketsim-viewer 39202
```

In the program above, replace `arena.try_set_vis_enabled(true)?` with:

```rust
arena.try_set_net_vis(Some("127.0.0.1:39202"))?;
```

`try_set_net_vis` never starts a viewer. Pass `None` to disconnect:

```rust
arena.try_set_net_vis(None)?;
```

Each viewer accepts one arena. Use a separate viewer process and port for each
arena. A second arena in the same process gets `AddrInUse` if it tries to use
an address already attached to another arena.

Addresses must be numeric IP-and-port addresses. Host names are not resolved.
The viewer binds to loopback by default. The protocol has no authentication.
Use non-loopback addresses only on trusted networks, and restrict access with
a firewall.

## Apply viewer controls

Sending frames does not let the viewer change your simulation. Your
application decides whether to apply controls, actions, and mutators.

To let the viewer drive the single car in the program above, remove the fixed
throttle assignment. Put this code before `arena.step_tick()` in the loop:

```rust
let controls = arena.latest_vis_controls()
    .map_or_else(CarControls::default, |message| message.controls);
arena.set_car_controls(car, controls);
```

`latest_vis_controls` returns `None` when input is absent, disconnected, or
more than 250 ms old. The example sends neutral controls in that case.
For several cars, use the message's `car_index` to select the target. Release
the previous car's controls when the target changes.

`take_vis_actions` consumes pending requests: bit 0 requests a reset, bit 1
a dribble, and bit 2 a ball launch. Your application must implement those
requests. See [`examples/drive.rs`](examples/drive.rs) for a complete loop that
handles controls, actions, mutators, and timing.

## Retained debug rendering

After attaching a viewer, get a cloneable renderer with
`arena.debug_renderer()`. It returns `None` when the arena is not attached.
`arena.try_set_net_vis(Some(address))?` connects to an existing viewer and
returns address or connection setup errors. This method does not launch a
viewer.

This example replaces a named group, then changes only its transform. It
assumes `arena` is mutable and the connection is attached:

```rust
use rocketsim_vis::debug::{
    Color, DebugRenderer, DepthMode, DrawList, GroupOptions, GroupTransform,
    Lifetime, Width,
};

let renderer: DebugRenderer = arena.debug_renderer().ok_or("not attached")?;
let mut drawing = DrawList::new();
drawing.line(
    [0.0, 0.0, 0.0],
    [100.0, 0.0, 0.0],
    Color::RED,
    Width::pixels(2.0),
    DepthMode::Test,
);
renderer.replace_group(
    "target",
    drawing,
    GroupOptions::new().lifetime(Lifetime::Persistent),
)?;

renderer.set_group_transform(
    "target",
    GroupTransform::from_position([0.0, 100.0, 0.0]),
)?;
```

`DrawList` supports explicit lines, polylines, points, meshes, mesh instances,
labels, rectangles, and helpers for boxes, circles, spheres, arrows, grids, and
axes. See [the retained rendering guide](DEBUG_RENDERING.md) for group updates,
anchors, resources, limits, and current rendering limits.

Use `DebugRenderer::draw_frame` for temporary marks. Use named groups for
persistent drawings. Both APIs use the same retained debug scene.

## Troubleshooting

- **Mesh loading fails:** Check the process's working directory. Use
  `rocketsim::init` with an explicit mesh directory if necessary. Handle
  initialization errors before you create an arena.
- **The viewer executable is not found:** Install it, build it, or set
  `ROCKETSIM_VIEWER` to its executable path. A workspace build alone does not
  add the binary to `PATH`. The launcher also checks local build locations
  beside the current executable before it searches `PATH`.
- **No scene appears:** Check the viewer address and confirm that your
  application calls `step_tick`. A connection alone does not send new frames.
- **Viewer input has no effect:** Apply the received controls in your loop.
  In the viewer, select a car, close settings, and disable spectate-only mode.
- **A second arena gets `AddrInUse`:** Use another viewer process and port.

## Compatibility

Keep `rocketsim`, `rocketsim_vis`, and `rocketsim_viewer` on matching versions.
Version 0.3 uses protocol version 1. Use matching builds of `rocketsim_vis`
and `rocketsim_viewer`. The installed viewer does not need collision meshes
because the simulation sends scene geometry.

The old `backend`, `camera`, and `VisInst` interfaces are no longer available.
Use `ArenaVisExt` for simulation connections and `debug::DebugRenderer` for
drawing. `proto::DebugDraw` and `VisFrame.debug` are removed. Do not use the
standalone frame codecs or control ports.

Custom viewer code uses `transport::viewer::ViewerListener::{bind, try_bind}`
and reads a unified `ViewerSnapshot`. Its fields include the generation,
connection status, optional `Arc<VisFrame>`, scene metadata, retained debug
scene, and counters. This replaces the old `FrameClient` API. `ArenaVisExt` and
`NetVis` remain the simulation-facing connection APIs. Viewer camera code
belongs to `rocketsim_viewer`.

The Rust API and the wire format changed during development. The protocol
version remains `1` because this format is not released. Rebuild the
simulation and viewer together. Earlier development binaries are not
compatible.

See the [API guide](https://docs.rs/rocketsim_vis) for complete examples of
connections, controls, and timing. The
[debug-rendering API](https://docs.rs/rocketsim_vis/latest/rocketsim_vis/debug/)
includes tested examples for frame drawings, persistent groups, reusable
meshes and textures, lifetimes, and handle errors.
