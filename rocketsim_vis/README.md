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

Viewer window (needs focus; input is read in-window so it works on
Wayland):

* `C` — cycle camera: wide view, then each car in order
* `Space` — toggle ball cam while following a car
* `1-8` — follow a car, `9` director, `0` auto cam, `F` free camera
* WASD + mouse — drive the car you follow (other views send neutral)
* `Backspace` or the settings-menu "Reset play" button — reset to kickoff
* `Esc` — options menu (sliders, camera presets, overlays)

While playing, the mouse is captured by the 3D view; it returns when the
menu opens or the window loses focus. An unfocused window never drives.

The driveable demo (`examples/drive.rs`) additionally uses, in the viewer
window: Q/E roll, Shift handbrake, `F2` dribble the ball, `F4` launch it.
Inputs stream to the sim over the control channel (`127.0.0.1:39202`);
gamepads keep working from the sim side with no focus needed.

## Examples

```sh
# Smallest streaming sim, no input (launches vis_viewer if needed)
cargo run -p rocketsim_vis --example minimal
# Interactive driveable demo (drive from the viewer window)
cargo run -p rocketsim_vis --example drive
# Deterministic scene for screenshot review
cargo run -p rocketsim_vis --example photo
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
