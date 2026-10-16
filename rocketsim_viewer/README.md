# rocketsim_viewer

3D visualizer for [RocketSim](https://github.com/ZealanL/RocketSim) games (eframe + egui), run as the `rocketsim-viewer` binary.

Enable it on any `rocketsim::Arena` and a window opens on a background thread,
following the cars, ball, and boost pads while you keep stepping ticks.

## Install / run

```sh
cargo install rocketsim_viewer
cargo binstall rocketsim_viewer
rocketsim-viewer
# ...or from this repo without installing:
cargo run -p rocketsim_viewer --bin rocketsim-viewer
```

Prebuilt `cargo binstall` assets are published on GitHub releases when available.

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

New sim code should depend on the lightweight `rocketsim_vis` crate (as
above); this crate re-exports its network API (`rocketsim_viewer::ArenaVisExt`,
`proto`, ...) so existing imports keep compiling unchanged.

## Controls

Viewer window (needs focus; input is read in-window so it works on
Wayland):

* `C` — cycle camera: wide view, then each car in order
* `Space` — toggle ball cam while following a car
* `1-8` — follow a car, `9` director, `0` auto cam, `F` free camera
* WASD + mouse — drive the car you follow (other views send neutral)
* `Backspace` or the settings-menu "Reset play" button — reset to kickoff
* `Esc` — options menu (sliders, camera presets, overlays)
* Pad: Options/Start toggles the menu · D-pad/left stick moves ·
  A selects · B backs out · ◀▶ adjust sliders · right stick looks

While playing, the mouse is captured by the 3D view; it returns when the
menu opens or the window loses focus. An unfocused window never drives.

## Examples

```sh
# Smallest streaming sim, no input (launches rocketsim-viewer if needed)
cargo run -p rocketsim_vis --example minimal
# Interactive driveable demo (drive from the viewer window)
cargo run -p rocketsim_vis --example drive
# Hoops: the viewer follows the sim's mode on its own (its -g only sets
# the pre-connect mesh)
cargo run -p rocketsim_vis --example drive -- -g hoops
```

## Assets

Car/ball/arena models are by ZealanL (see `src/models/MODEL_LICENSE_README.md`):
credit ZealanL and don't sell them.
