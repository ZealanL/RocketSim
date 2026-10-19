# Retained debug rendering

Use `rocketsim_vis::debug` for persistent debug geometry. Attach the arena to a
viewer before you get a renderer:

```rust
arena.try_set_net_vis(Some("127.0.0.1:39201"))?;
let renderer = arena.debug_renderer().ok_or("viewer is not attached")?;
```

`try_set_net_vis` accepts a numeric IP address and port. It does not start a
viewer. Pass `None` to detach. It returns `std::io::Result` for invalid
addresses and attach errors. `debug_renderer()` returns `Option<DebugRenderer>`.

## Groups and updates

Use the Canvas API for common drawing. `draw_frame` replaces the short-lived
`debug/frame` group. `GroupHandle::draw` replaces a persistent named group.

```rust
renderer.draw_frame(|draw| {
    draw.line([0.0; 3], [100.0, 0.0, 0.0])
        .color(Color::RED)
        .width_px(2.0);
    draw.text_2d([0.03, 0.04], "Target");
})?;

renderer.group("planner/target").draw(|draw| {
    draw.line([0.0; 3], [100.0, 0.0, 0.0])
        .color(Color::RED)
        .width_px(2.0);
})?;
```

`draw_frame` expires on the next displayed snapshot after a simulation tick.
It does not expire after a 250 ms wall-clock timeout. While the simulation is
paused, the frame drawing remains. Canonical retained updates are independent
of simulation ticking. `DebugRenderer::snapshot` reuses one `Arc<DebugScene>`
until a scene change or expiry. It checks the earliest tick and Unix-time
deadlines before it scans groups. Persistent-only scenes need no expiry scan.

`Lifetime::Seconds` uses an absolute Unix-millisecond deadline. It is intended
for a local viewer. Clock differences between machines can change when a
remote viewer expires the drawing.

`Canvas` provides fluent `line`, `polyline`, `point`, `text_2d`, `text_3d`, and
`rect` helpers. It sets defaults that you can change with builder methods.
Raw `DrawList` helpers remain available for explicit primitives. These include
`mesh`, `instance`, `label_screen`, `label_world`, `box_edges`, `box_solid`,
`circle`, `sphere_lines`, `sphere_solid`, `arrow`, `grid`, and `axes`.

Replace a named group to replace its complete geometry:

```rust
use rocketsim_vis::debug::{
    Color, DepthMode, DrawList, GroupOptions, GroupTransform, Lifetime, Width,
};

let mut list = DrawList::new();
list.line(
    [0.0, 0.0, 0.0],
    [100.0, 0.0, 0.0],
    Color::RED,
    Width::pixels(2.0),
    DepthMode::Test,
);
renderer.replace_group(
    "target",
    list,
    GroupOptions::new().lifetime(Lifetime::Persistent),
)?;

renderer.set_group_transform(
    "target",
    GroupTransform::from_position([0.0, 100.0, 0.0]),
)?;
```

`replace_group` returns a revision. It accepts `DrawList` and `GroupOptions` and
returns `DebugResult`. It creates or replaces the group with that stable name.
`GroupHandle` provides the same operations for one group. A transform update
changes placement without resending geometry or restarting the group's
lifetime.

Transforms apply in this order: item local transform, instance transform,
group scale, group rotation, group translation, anchor offset, then the live
anchor frame. Group rotation matrices use row-major order. `Anchor::World` is
fixed. `Anchor::Ball` and `Anchor::Car` follow the latest received frame.
Anchors use stable identifiers, not pointers into simulation state.

Use `MeshResource` and `TextureResource` for reusable named resources. Insert
or update a resource once, then refer to it by ID in mesh instances or textured
items. The transport sends resource bytes separately from group updates. A
transform-only change does not resend mesh geometry. Geometry replacement
updates the group's geometry revision. Treat item handles as stale after any
geometry edit; reacquire items after replacement. Handle operations return a
`DebugResult` and report stale or missing handles rather than silently
changing another item.

## Built-in assets

Use `BuiltinMesh` and `BuiltinTexture` for assets that the viewer provides.
Call `id()` to get the reserved identifier. A built-in reference does not need a
`MeshResource` or `TextureResource`.

`BuiltinMesh` provides `@builtin/ball`, `@builtin/puck`,
`@builtin/car_octane`, `@builtin/car_dominus`, `@builtin/car_breakout`, and
`@builtin/car_merc`. `BuiltinTexture` provides `@builtin/ball`,
`@builtin/car_blue`, and `@builtin/car_orange`.

Do not create a user resource with an identifier that starts with `@builtin/`.
Other mesh and texture identifiers must refer to inserted user resources.

```rust
use glam::Mat3A;
use rocketsim_vis::debug::{BuiltinMesh, BuiltinTexture, Color, DebugRenderer, DepthMode, DrawList, GroupOptions};

let renderer = DebugRenderer::new_detached();
let mut draw = DrawList::new();
draw.instance(
    BuiltinMesh::Ball.id(), [0.0; 3], Mat3A::IDENTITY.to_cols_array_2d(),
    [1.0; 3], Color::WHITE, Some(BuiltinTexture::Ball.id().into()),
    DepthMode::Test,
);
renderer.replace_group("ball", draw, GroupOptions::new())?;
```

## Depth and transparency

Use `DepthMode::Test` for geometry that opaque objects must hide. Use
`DepthMode::Always` only for drawing that must remain visible over the world.
`DepthMode::Xray` shows hidden geometry with reduced opacity.

The viewer draws opaque geometry before transparency. It sorts translucent
mesh instances back-to-front across groups. Only adjacent compatible
instances share a draw call. Unchanged geometry stays cached. The viewer
updates the sorted instance data when the camera or placement changes.

Sorting uses instance centers. Intersecting translucent surfaces can still
need triangle-level sorting or another transparency method.

`DrawList::grid` draws on the XY ground plane at the supplied Z position.
The OBJ demo guide uses depth testing so that it cannot cover the model.

## Lifetimes and validation

`Lifetime::Persistent` remains until replacement or clear. `Lifetime::Frame`
lasts for a streamed frame. `Lifetime::Ticks(n)` expires by simulation ticks.
`Lifetime::Seconds(s)` expires by wall-clock time. Tick lifetimes accept 1 to
1,000,000 ticks. Seconds lifetimes accept 0.001 to 3600 seconds. Transform and
anchor updates do not restart a lifetime.

Validation returns `DebugResult` for invalid names, non-finite values, invalid
colors, widths, transforms, anchors, missing resources, stale handles, and
resource or count limits. The default total retained debug payload limit is
8 MiB. Other limits include group, item, mesh, texture, text, and geometry
bounds. Check the exported `MAX_*` constants for exact values. Validation can
reject an update before it reaches the viewer.

## Implemented limits

This API does not implement dashed lines, separate 2D line primitives, or
oriented world text. Picking is not implemented. World labels render as an
overlay and are not depth-tested. Labels support horizontal alignment only;
the current overlay uses vertical center alignment.

## Example and review tool

`examples/debug_render.rs` is the CLI driver. Scene builders live in
`examples/support/debug_render/`. The example connects to an already running
viewer with `--addr IP:PORT`. Its modes are `wire`, `solid`, `showcase`, and
`stress`.
It also accepts `--drive`, `--seconds`, `--mode`, `--count`, `--obj`, `--static`,
`--original-placement`, and `--benchmark-output`. Run `--help` for exact CLI
syntax. Debug-only runs stop after 15 seconds by default.

Start the viewer with the simulation ball hidden for these debug-only scenes:

```sh
./target/release/rocketsim-viewer --hide-ball --no-menu
```

The demo keeps the simulation ball at the camera target. Its mesh can
intersect the displayed model. `--hide-ball` hides that mesh without changing
physics or camera tracking.

The `wire` and `solid` modes need `--obj PATH`. The review harness expects a
user-local ZeroTwo OBJ file for these modes. No model or derived asset is in
the repository because its license is unknown. The procedural showcase uses
original code and no third-party assets.

### Drive around the debug scene

Add `--drive` to any mode to use a Soccar arena with an Octane. This mode
requires the Soccar collision meshes. It uses the same controls, actions,
and mutators as the `drive` example. The ball uses normal simulation physics.
Debug geometry is visual only. Cars and the ball pass through it.

Run the procedural scene from the workspace root:

```sh
cargo run --locked --release -p rocketsim_vis --example debug_render -- \
  --drive --mode showcase
```

Use a local OBJ instead:

```sh
cargo run --locked --release -p rocketsim_vis --example debug_render -- \
  --drive --mode solid --obj /path/to/ZeroTwo.obj --static
```

With `--drive`, omit `--addr` to build and start the workspace viewer. Use
`--addr IP:PORT` to connect to an existing viewer. Keep the ball visible for
normal play. Restart a viewer that was started with `--hide-ball` if you want
to see the ball.

Close settings with `Escape`. Press `1` to follow and drive the Octane. Press
`F` for free camera. Use `W`, `A`, `S`, `D`, and the mouse to move the camera.
Press `1` to return to the car. These are the default bindings.

`--static` stops debug-scene animation. It does not stop the simulation.
Driving runs until you press `Ctrl+C` in the terminal. Add `--seconds N` for a
timed run. Resetting play does not remove the debug scene.

### Capture a scene

From the workspace root, run the harness with:

```sh
python3 tools/debug-render-review.py --only showcase --only stress \
  --viewer-arg=--no-menu
```

`--only` selects one or more modes. `--static` keeps the selected scene static.
Pass `--obj PATH` for wire or solid mode. Repeat `--viewer-arg=ARG` to pass
viewer options. The harness hides the simulation ball by default. Pass
`--show-ball` to include it. Pass `--drive` to capture the Soccar scene with
its car and ball. The harness adds `--debug-stats PATH` to collect
CPU timings and asynchronous GPU query statistics. It saves captures and logs
under `target/input-review/debug-render`. No measurement is stated here.

Matching `rocketsim_vis` and `rocketsim_viewer` builds use protocol version 1.
