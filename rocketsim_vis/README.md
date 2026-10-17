# rocketsim_vis

Lightweight interface between RocketSim and the detached viewer. It streams
simulation frames and viewer controls over one versioned, bidirectional TCP
connection. It has no window or renderer. Simulation crates use this crate,
not `rocketsim_viewer`.

## Quick start

```rust,no_run
use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
use rocketsim_vis::ArenaVisExt;

init_from_default(true).unwrap();
let mut arena = Arena::new(GameMode::Soccar);
arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
arena.reset_to_random_kickoff(None);
arena.set_vis_enabled(true);

for _ in 0..240 {
    arena.step_tick();
    std::thread::sleep(std::time::Duration::from_millis(8));
}
arena.set_vis_enabled(false);
```

The default viewer endpoint is `127.0.0.1:39201`. The simulation dials the
viewer. The viewer sends controls on the same connection. The stock viewer
does not use a separate port `39202` control connection.

`set_vis_enabled(true)` launches or attaches to the default endpoint.
`set_net_vis(Some(address))` connects to an explicit endpoint and never
launches a viewer. Use `try_set_vis_enabled` and `try_set_net_vis` to handle
errors. The non-`try` methods report errors to stderr. Each viewer accepts one
arena. A second arena in one process cannot attach to an address already in
use; it gets `AddrInUse`. Use separate viewer instances and explicit ports.

The launcher searches `PATH` and local build locations for
`rocketsim-viewer`. Set `ROCKETSIM_VIEWER` to choose an executable path. The
viewer is a separate crate. Build or install it separately from the simulation.

Addresses must be numeric socket addresses. Host names are not resolved. The
viewer binds to loopback by default. Use `--bind` to listen on another address.
A non-loopback bind can expose the viewer to untrusted peers. The protocol has
no authentication. Bind only on a trusted network and restrict access with
your network firewall.

The simulation sends scene mesh metadata with its frames. This lets an
installed viewer render without a local collision-mesh tree. The simulation
must still initialize its own meshes before it steps the arena.

## Controls and debug drawing

The `ArenaVisExt` network API provides `latest_vis_controls(&self) ->
Option<ControlMsg>`, `take_vis_actions(&mut self) -> u8`, and
`set_vis_debug(&mut self, DebugDraw)`. `FrameClient::send_controls` sends
controls over its accepted frame connection. The older `ControlServer` API
may remain for compatibility. The stock viewer does not need that separate
server.

`DebugDraw` has `lines`, `text`, `rects`, and `models` lists. Lines use 3D
world-space endpoints, RGBA color values, and a width. Text uses normalized
screen coordinates from 0 to 1 and an RGBA color. Rectangles use normalized
screen-space `min` and `max` points and an RGBA color. Models use a model name,
a texture name, a 3D position, a 3-by-3 rotation matrix, and a 3D scale. Names
must match assets in the viewer scene. For example, add a line, screen text,
and a stock Octane model like this:

```rust
use rocketsim_vis::proto::{DebugDraw, DebugLine, DebugModel, DebugText};

let mut debug = DebugDraw::default();
debug.lines.push(DebugLine {
    start: [0.0, 0.0, 0.0],
    end: [100.0, 0.0, 0.0],
    color: [1.0, 0.0, 0.0, 1.0],
    width: 2.0,
});
debug.text.push(DebugText {
    text: "Example".into(),
    position: [0.05, 0.05],
    color: [1.0, 1.0, 1.0, 1.0],
});
debug.models.push(DebugModel {
    model: "car_octane".into(),
    texture: "car_blue".into(),
    position: [0.0, 0.0, 100.0],
    rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    scale: [1.0, 1.0, 1.0],
});
arena.set_vis_debug(debug);
```

Debug data is sent as part of subsequent streamed frames.

## Wire compatibility and migration

Protocol version 3 has a wire compatibility break. Version 2 viewers and
simulations cannot communicate with this version. The old backend,
camera, and `VisInst` interfaces were removed. Use `DebugDraw` for lines,
text, rectangles, and models.

Use `rocketsim_vis` for new network code and `rocketsim_viewer` for the
viewer. The viewer still re-exports network items, so existing
`rocketsim_viewer::` imports remain supported. This release has a major wire
protocol compatibility break. Keep both crates on matching versions.
