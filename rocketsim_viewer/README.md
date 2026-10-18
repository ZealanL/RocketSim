# rocketsim_viewer

3D visualizer for RocketSim. The `rocketsim-viewer` binary accepts one
simulation connection. One versioned, bidirectional TCP connection carries
frames and controls. The default endpoint is `127.0.0.1:39201`. The stock
viewer does not use a separate `39202` control channel.

## Build and run

The workspace examples build and launch the matching viewer:

```sh
cargo run --release --example drive
cargo run -p rocketsim_vis --example minimal
```

The first run also compiles the graphics dependencies. Later runs update the
viewer only when needed. `ROCKETSIM_VIEWER` selects an executable and skips
this build. `drive --viewer IP:PORT` connects without building or launching a
viewer.

To run the viewer directly:

```sh
cargo run -p rocketsim_viewer --bin rocketsim-viewer
```

Install from source with:

```sh
cargo install rocketsim_viewer
```

On Linux, a source build with gamepad support needs the `libudev` development
package. A source install also needs the Rust toolchain and system libraries
required by the graphics stack. The launcher looks for a local build of
`rocketsim-viewer` before it searches `PATH`. Set `ROCKETSIM_VIEWER` to an
executable path to select a specific viewer binary. Do not assume that
prebuilt release files are available. The viewer and simulation are separate
crates. Applications other than the workspace examples must build or install
the viewer separately from the simulation.

## Sim integration

Add `rocketsim_vis` to the simulation and use `ArenaVisExt`:

```rust,no_run
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

The simulation sends scene mesh metadata with frames. The viewer uses this
data to render without a local collision-mesh tree. The simulation must still
initialize its own meshes. When the stream disconnects, the viewer clears the
frame and resets to its placeholder scene. A later simulation connection can
send a new frame and scene.

Use separate viewer instances and explicit ports for multiple arenas. Each
viewer accepts one simulation connection. Two arenas in one process cannot
attach to the same viewer address; the second attach returns `AddrInUse`.

`set_vis_enabled(true)` launches or attaches to the default viewer endpoint.
It does not select a custom endpoint. `set_net_vis(Some(address))` connects to
an explicit endpoint and never launches a viewer. Use the `try_set_*` methods
to handle startup and address errors. The non-`try` methods report errors to
stderr.

## CLI and network access

`rocketsim-viewer [PORT] [CAM] [-g MODE] [--bind ADDRESS] [--gpushot]` listens
on port `39201` by default. `ADDRESS` must be a numeric bind address. The
viewer binds to `127.0.0.1` by default. A non-loopback bind can expose the
viewer to untrusted peers. The protocol has no authentication. Bind only on a
trusted network and restrict access with a firewall. Simulation endpoints
must also use numeric socket addresses; host names are not resolved.

`CAM` selects the initial camera: birds-eye (`birds`), a followed car
(`followN`), director, auto camera (`autoN`), or free camera. `N` is a zero-based
car index. If the car is absent, the viewer uses wide view. Selection takes
constant time, including for a large index. `-g MODE` selects the initial placeholder mode. Stream
metadata replaces it when a simulation connects. `--gpushot` enables GPU-stall
timing.

## Controls and settings

All controls use one action map. It has 56 actions, with two input slots for
each action. Each slot can use a keyboard key, mouse input, or controller input.
The settings menu has Camera, Bindings, Display, and Simulation tabs. The
Bindings tab has Keyboard & Mouse, Controller, and All views. These views
filter the same two slots. They do not add slots or restrict input capture.
Use All when both slots have assignments on another device.

Search for an action or select an action group. Clear a slot or reset the
visible device assignments. A device reset preserves hidden assignments.
Duplicate bindings show a warning; they are allowed. Reset all settings
requires confirmation.

Supported inputs include egui keyboard keys and modifiers, five mouse buttons,
signed X/Y mouse motion and wheel input, controller buttons and triggers,
signed stick directions, and raw controller codes. Capture accepts any input
the viewer receives, including Escape. Use the Cancel button to stop capture.
Tap the current Back input to bind it. Hold it for 0.75 seconds to cancel.
Operating-system-reserved keys may not reach egui. The settings menu shows
capture status in its action label.

Defaults include `W`/`S`/`A`/`D` driving and free-camera movement, `Space` for
free-camera up, and `Shift` for handbrake, air roll, and free-camera down,
and `Ctrl` for fast free-camera movement. The mouse buttons jump and boost.
Mouse movement looks in free-camera mode; the wheel moves the free camera in
and out. `R` resets it. Camera defaults are `1`-`8` to follow cars, `9` for
Director, `0` for Auto, `F` for free camera, `C` to cycle, and `Space` to
toggle ball camera while following.

The controller defaults use the left stick for driving and free-camera movement,
the right stick for look, and the triggers for throttle/brake and free-camera
up/down. Follow-camera look and free-camera look have separate bindings. The left thumb button moves the free camera fast; the right thumb
button resets it. Controller selection prefers a real mapped controller over a
system controller. You can change the selected controller in settings.

Input actions work only while the viewer window has focus. The menu blocks
driving input. The viewer sends neutral drive controls when spectate-only mode
is enabled. This mode does not apply inputs from any other unfocused window.

`Auto` selects automatic ball-camera behavior. `Director` chooses a target with
a deterministic, short-horizon ballistic and floor-bounce heuristic. It aims
to frame predicted shots and saves and to select player views. It is not an
implementation of Rocket League's proprietary Director AI and does not promise
matching cuts or predictions. Rocket League describes its Director Mode as
using match analysis to place and move cameras and select player views:
[official Director Mode overview](https://www.rocketleague.com/en/news/autumn-update-local-matches-director-mode).
Its [v1.37 patch notes](https://www.rocketleague.com/news/patch-notes-v1-37)
describe prediction of shots, goals, and saves.

New bindings are saved in `[action_bindings.bindings]`. Each action value has
two source slots. For example:

```toml
[action_bindings.bindings]
ThrottleForward = [
  { type = "PadAxis", value = { axis = "RightZ", sign = 1 } },
  { type = "Unbound" },
]
Jump = [
  { type = "Key", value = "Space" },
  { type = "Mouse", value = "Primary" },
]
```

This example shows the source-tagged format. Use the exact enum names written
by the viewer. The legacy `[bindings]` table is used only when
`[action_bindings]` is absent. Once the new table exists, legacy bindings do not
provide live actions. An action omitted from the new table is unbound. Use the
viewer to change one action and retain the other actions. The example above is
not a complete control map. This settings change does not change the network
wire format.

Settings use `settings.toml` in the OS config directory under
`rocketsim-vis`. Linux uses `$XDG_CONFIG_HOME/rocketsim-vis/` or, if unset,
`~/.config/rocketsim-vis/`. macOS uses
`~/Library/Application Support/rocketsim-vis/`. Windows uses
`%APPDATA%/rocketsim-vis/`. If no OS config directory is available, the viewer
uses `./settings.toml`. The viewer saves changed settings after a short delay
and flushes them when it exits.

The spectate-only setting sends neutral controls, zero action bits, and a zero
mutator mask. The viewer sends controls and mutator values; the simulation
chooses whether to apply them. The `drive` example applies the received
mutators.

## Examples

These workspace examples build the viewer before they launch it:

```sh
cargo run -p rocketsim_vis --example minimal
cargo run -p rocketsim_vis --example drive
cargo run -p rocketsim_viewer --example photo
cargo run -r -p rocketsim_viewer --example bench_frame
```

`drive` is an interactive simulation. `photo` builds a deterministic Soccar
scene and streams it for visual review. After it prints its 600-tick checksum,
it deliberately keeps simulating and streaming. Stop it with Ctrl-C.
`bench_frame` measures viewer frame work without opening a window.

## Migration

Use `rocketsim_vis` for new network code and `rocketsim_viewer` for the
viewer. The viewer still re-exports network items, so existing
`rocketsim_viewer::` imports remain supported. Version 0.3 uses wire protocol 3
because the serialized `CarState` layout changed. Protocol 2 viewers and
simulations cannot communicate with this version. Build both crates together. The old `rocketsim_vis::{backend, camera,
VisInst}` APIs were removed. Viewer camera code now lives in
`rocketsim_viewer`, but it is not a drop-in replacement for the old camera API.
Use `DebugDraw` for lines, text, rectangles, and models. See
[`rocketsim_vis`](../rocketsim_vis/README.md) for field details.
