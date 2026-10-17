# rocketsim_viewer

3D visualizer for RocketSim. The `rocketsim-viewer` binary accepts one
simulation connection. One versioned, bidirectional TCP connection carries
frames and controls. The default endpoint is `127.0.0.1:39201`. The stock
viewer does not use a separate `39202` control channel.

## Build and run

Build the viewer before you run lightweight examples, or install it so the
launcher can find `rocketsim-viewer` on `PATH`:

```sh
cargo build -p rocketsim_viewer --bin rocketsim-viewer
cargo run -p rocketsim_vis --example minimal
```

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
required by the graphics stack. The launcher looks for an installed
`rocketsim-viewer` on `PATH` or a local build. Set `ROCKETSIM_VIEWER` to an
executable path to select a specific viewer binary. Do not assume that
prebuilt release files are available. The viewer and simulation are separate
crates. Build or install the viewer separately from the simulation.

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

The keyboard camera defaults are `1`-`8` to follow cars, `9` for director,
`0` for auto camera, `F` for free camera, `C` to cycle, and `Space` to toggle
ball cam while following. Driving keys use WASD. Mouse movement looks around in free-camera mode. The
primary mouse button jumps; the secondary mouse button boosts. Rebind keyboard
and mouse driving actions, including jump and boost, in settings. Controller
bindings are configurable.

Controller actions require the viewer window to have focus. This includes
menu navigation. The right stick adjusts look direction while following a car.
It does not control free-camera movement. Controller navigation can reach all
menu sections.

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

Build the viewer first as shown above, or install it on `PATH` before running
examples that launch it:

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
`rocketsim_viewer::` imports remain supported. This release has a major wire
protocol compatibility break. Older viewer and simulation versions cannot
communicate with this version. The old `rocketsim_vis::{backend, camera,
VisInst}` APIs were removed. Viewer camera code now lives in
`rocketsim_viewer`, but it is not a drop-in replacement for the old camera API.
Use `DebugDraw` for lines, text, rectangles, and models. See
[`rocketsim_vis`](../rocketsim_vis/README.md) for field details.
