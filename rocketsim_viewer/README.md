# rocketsim_viewer

Use `rocketsim-viewer` to watch a RocketSim simulation, follow a car, move a
free camera, or test driving controls. The application displays frames sent
by a separate simulation. It does not run a match by itself.

To connect your own simulation, use
[`rocketsim_vis`](../rocketsim_vis/README.md). Your simulation does not need
the viewer's graphics dependencies.

## Run your first scene

From a RocketSim source checkout:

1. [Prepare the collision meshes](../README.md#prepare-the-collision-meshes).
2. Run the drive example from the workspace root:

   ```sh
   cargo run --release --example drive
   ```

The command builds the matching viewer and starts both applications. The
first build also compiles the graphics dependencies. Later runs rebuild the
viewer only when needed.

Click the viewer window. Close settings with `Escape` if the menu is open.
Press `1` to follow the first car, then use the driving controls below. Stop
the simulation with Ctrl-C in its terminal.

To watch a fixed-seed scene instead, run:

```sh
cargo run -p rocketsim_viewer --example photo
```

Build the viewer before you run the photo example. The example starts or
reuses that executable. Its fixture is in `examples/support/photo.rs`.
It prints state after 600 ticks, then keeps simulating and streaming until
you stop it with Ctrl-C.

## Install the viewer separately

Use this option when your simulation application is outside the workspace.
The viewer needs a desktop display and OpenGL 3.3 support. The simulation
needs collision meshes; an installed viewer does not. The simulation sends
arena geometry to the viewer.

To install a published `0.3.0` release from source:

```sh
cargo install --locked rocketsim_viewer --version 0.3.0
```

A source build needs your platform's graphics system libraries. On Linux,
install `pkg-config` and the `libudev` development package. On Debian or
Ubuntu, these packages are `pkg-config` and `libudev-dev`.

If you have [cargo-binstall](https://github.com/cargo-bins/cargo-binstall)
and a matching prebuilt release is available, use:

```sh
cargo binstall rocketsim_viewer --version 0.3.0
```

Prebuilt targets are Linux x86-64, Windows x86-64, and macOS on Intel and
Apple Silicon. Linux releases require glibc 2.35 or later and desktop graphics
libraries. macOS releases target macOS 11 or later. Other targets can compile
from source.

Start the installed viewer with:

```sh
rocketsim-viewer
```

It waits for one simulation at `127.0.0.1:39201`. Follow the
[simulation connection guide](../rocketsim_vis/README.md) to send frames.
Installing `rocketsim_vis` does not install the viewer application.

From a source checkout, you can start only the viewer with:

```sh
cargo run -p rocketsim_viewer --bin rocketsim-viewer
```

## Controls

Input actions work only while the viewer window has focus. The settings menu
blocks driving input. To drive a car, follow it with `1` through `8`. The
simulation must accept viewer controls. The `drive` example does this; the
`photo` example does not.

### Keyboard and mouse defaults

| Action | Input |
| --- | --- |
| Throttle and reverse | `W` / `S` |
| Steer | `A` / `D` |
| Jump | Primary mouse button |
| Boost | Secondary mouse button |
| Handbrake and air roll | `Shift` |
| Air roll left / right | `Q` / `E` |
| Follow cars 1 through 8 | `1` through `8` |
| Toggle ball camera while following | `Space` |
| Director / Auto camera | `9` / `0` |
| Free camera / cycle camera mode | `F` / `C` |
| Move the free camera | `W`, `A`, `S`, `D` |
| Move the free camera up / down | `Space` / `Shift` |
| Move the free camera faster | Hold `Ctrl` |
| Look around in free camera | Move the mouse |
| Move the free camera forward / backward | Mouse wheel |
| Reset the free camera | `R` |
| Open or close settings | `Escape` |
| Reset play / set up a dribble / launch the ball | `Backspace` / `F2` / `F4` |

The last row sends requests to the simulation. Your simulation decides what
each request does.

### Controller defaults

Select a car with `1` through `8`. For controller-only use, bind a Follow
action to a controller input in **Bindings**. Controller button names below
use their physical positions.

| Action | Input |
| --- | --- |
| Throttle / reverse | Right / left trigger |
| Steer and control the car in the air | Left stick |
| Jump / boost | Bottom / right face button |
| Handbrake and air roll | Left face button |
| Air roll left / right | Left / right shoulder button |
| Toggle ball camera | Top face button |
| Look around | Right stick |
| Move the free camera | Left stick |
| Move the free camera up / down | Right / left trigger |
| Move the free camera faster | Hold the left stick button |
| Reset the free camera | Right stick button |
| Open or close settings | Start |

Select the controller in settings if the viewer chooses a different device.

### Camera modes

Follow mode keeps one car in view. Ball camera turns that view toward the
ball. Free camera lets you move without following a car. Auto changes
ball-camera behavior automatically. Director selects views based on predicted
play, including possible shots and saves. It is an approximation, not Rocket
League's Director implementation. Do not expect identical camera cuts.

## Change a binding

1. Press `Escape` to open settings.
2. Open **Bindings**.
3. Select **Keyboard & Mouse**, **Controller**, or **All**.
4. Find the action. Select the input slot you want to change.
5. Press the new button or key, or move the desired axis.

Each action has two input slots in total. The device tabs show the same
slots; they do not add slots for each device. Use **All** if the slots contain
inputs from another device. Each slot accepts keyboard, mouse, or controller
input, regardless of the selected tab. Mouse motion, wheel directions,
controller triggers, stick directions, and raw controller codes are supported.
The viewer can only bind inputs that the operating system sends to it.

Use **Cancel** to stop capture. To bind the current Back input, tap it.
Hold that input for 0.75 seconds to cancel instead. You can clear a slot or
reset the visible device assignments. A device reset keeps assignments hidden
by that view. Duplicate bindings are allowed and show a warning.

The **Camera**, **Display**, and **Simulation** tabs contain the other
settings. Spectate-only mode sends neutral driving controls and no action
requests. The viewer sends mutator values, but the simulation decides whether
to apply them.

## Settings file

The viewer saves changes automatically after a short delay. It saves pending
changes when it exits. The file is `settings.toml` in the following directory:

| System | Directory |
| --- | --- |
| Linux | `$XDG_CONFIG_HOME/rocketsim-vis/`, or `~/.config/rocketsim-vis/` if unset |
| macOS | `~/Library/Application Support/rocketsim-vis/` |
| Windows | `%APPDATA%/rocketsim-vis/` |

If no operating-system configuration directory is available, the viewer uses
`./settings.toml`. Use the Bindings menu for normal changes. If you edit the
file, keep the complete `[action_bindings]` map. An omitted action is unbound;
it does not inherit a default binding. The old `[bindings]` table is used only
when `[action_bindings]` is absent.

## Use another port or machine

Use a separate viewer process and a separate port for each arena. For example:

```sh
rocketsim-viewer 39202
```

Connect the simulation to `127.0.0.1:39202` with `try_set_net_vis`. This method
does not start a viewer. The `drive` example can connect to it with:

```sh
cargo run --release --example drive -- --viewer 127.0.0.1:39202
```

The viewer binds to `127.0.0.1` by default. Use `--bind` with a local IP address
to listen on another interface. Addresses must be numeric; host names are not
resolved. The protocol has no authentication. Use a non-loopback address only
on a trusted network, and restrict access with a firewall.

Run `rocketsim-viewer --help` for all command options, including the initial
camera and game mode. Streamed scene data replaces the initial game mode.

## Viewer library modules

The viewer separates CPU geometry, rendering, effects, and input. The old flat
module paths are removed. Import CPU helpers from
`geometry::{frustum, lines, mesh, obj, pads}`. Import OpenGL pipelines from
`render::{main, lines, scene, view, debug}`. Simulation effects are in
`effects::{fx, dropshot}`. Input backends use `input::{sources,legacy}`.
`camera` and `settings` remain public root modules. `app` and `ui` are private.

The receiver API is `rocketsim_vis::transport::viewer::ViewerListener`. It
returns a `ViewerSnapshot` with an optional `Arc<VisFrame>`. The sender is
`rocketsim_vis::transport::simulation`; `NetVis` and `ArenaVisExt` remain at the
`rocketsim_vis` root. `FrameClient`, standalone frame codecs, and control ports
are removed. The unreleased wire format also changed. The protocol version
remains `1`. Rebuild the simulation and viewer together.

## Debug rendering review

The `debug_render` example shows the retained drawing API. Build and run it
with a viewer from the same checkout. See
[`rocketsim_vis/DEBUG_RENDERING.md`](../rocketsim_vis/DEBUG_RENDERING.md) for
its modes, local OBJ input, built-in debug assets, API limits, and review tool.

Pass `--debug-stats PATH` to write CPU timing and asynchronous GPU query
statistics to a file. Pass `--no-menu` to start with the menu closed. Pass
`--hide-ball` to hide the simulation ball mesh without changing physics or
camera tracking. Use this for debug-only model scenes. The viewer accepts
these options with its normal port and camera arguments.

## Troubleshooting

- **No scene appears:** The viewer is waiting for frames. Start a simulation
  and check its connection address and port.
- **The viewer does not start:** Check the graphics libraries and OpenGL
  support. For automatic startup, install the binary or set
  `ROCKETSIM_VIEWER` to its executable path. That variable also skips the
  viewer build in workspace examples.
- **Controls have no effect:** Focus the window, close settings, select a car,
  and disable spectate-only mode. Confirm that the simulation applies input.
- **A second arena cannot connect:** Start another viewer on a different port.
- **The connection reports a version mismatch:** Use matching builds of
  `rocketsim_vis` and `rocketsim_viewer`. Version 0.3 uses protocol version 1.
- **The scene disappears after a disconnect:** This is expected. The viewer
  clears the old scene and waits for a new simulation connection.

The viewer models have [separate credit and use requirements](src/models/MODEL_LICENSE_README.md).
Release archives include those model terms in `MODEL_LICENSE_README.md`.
