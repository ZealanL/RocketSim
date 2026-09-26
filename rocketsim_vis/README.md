# rocketsim_vis

3D visualizer for [RocketSim](https://github.com/ZealanL/RocketSim) games (miniquad + egui).

Enable it on any `rocketsim::Arena` and a window opens on a background thread,
following the cars, ball, and boost pads while you keep stepping ticks.

## Quick start

```rust
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, GameMode, Team, init_from_default};
use rocketsim_vis::ArenaVisExt;

init_from_default(true).unwrap();
let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar));
arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
arena.reset_to_random_kickoff(None);
arena.set_vis_enabled(true);

for _ in 0..240 {
    arena.step_tick();
    std::thread::sleep(std::time::Duration::from_millis(8));
}
```

Disabling is symmetric: `arena.set_vis_enabled(false)` drops the window.
Calling it with the current state is a no-op.

## Controls

Window (handled every tick):

* `C` — cycle camera: birds-eye, then each car in order
* `Space` — toggle ball-cam while following a car

The driveable demo (`examples/vis.rs`) additionally uses:
WASD + mouse to drive, Q/E roll, Shift handbrake, `Backspace` reset to
kickoff, `2` dribble the ball, `4` launch it.

## Examples

```sh
# Smallest windowed sim, no input
cargo run -p rocketsim_vis --example minimal
# Interactive driveable demo
cargo run -p rocketsim_vis --example vis
```

## Camera

`camera::CameraMan` holds the persistent view state (birds-eye or one
`camera::CarCam`). Tune it with `camera::CameraConfig` /
`camera::CarCameraConfig` (follow distance/height, ball-tilt curve, FOV,
birds-eye position). Note: `VisInst` currently uses `CameraConfig::default()`.

## Advanced: custom rendering

`VisInst` rebuilds a `backend::VisRenderState` each tick (models, lines,
overlay text, 2D shapes) and shares it with the renderer thread. To draw
your own overlay, implement `rocketsim::Vis` and push into `objects`,
`lines` (`add_line_simple`), `info_text_lines`, or `shapes_2d`
(`backend::Elem2D`, `backend::Color`).

`backend` also exposes the miniquad plumbing (`Model`/`ModelSet`,
`Texture`/`TextureSet`, `ShaderSrc`, `VisRenderer`, `WindowEvent`) if you
want to build a fully custom viewer. `vis_asset_loader` bundles the
built-in models/textures and lists their known names.

## Assets

Car/ball/arena models are by ZealanL (see `src/models/MODEL_LICENSE_README.md`):
credit ZealanL and don't sell them.
