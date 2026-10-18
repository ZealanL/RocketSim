# Viewer review tools

Use these tools to review viewer screenshots and test input integration.
The standalone `input-control-probe` crate receives viewer controls and
applies them to a test simulation. These tools are for maintainers.
Applications do not need them to run the viewer.

## Requirements

Run the commands from the workspace root. First
[prepare the collision meshes](../README.md#prepare-the-collision-meshes)
for the test simulation.

Screenshot and input review require:

- Linux and access to a desktop display for Xephyr.
- `Xephyr`, `xwininfo`, and `xwd`. Distribution packages include
  `xserver-xephyr`, `x11-utils`, and `x11-apps` on Debian or Ubuntu.
- Python 3.12 or later, Pillow (`PIL`), and NumPy.
- The X11 and XTest shared libraries: `libX11.so.6` and `libXtst.so.6`.

Virtual controller tests also need the Python `evdev` package and permission
to create a uinput device. Configure uinput access before you use
`--controller`. Do not change permissions on host input devices or take
exclusive control of those devices.

Build the probe and viewer from the same checkout. Version 0.3 uses protocol
version 1. The viewer listens on loopback by default. Its protocol has no
authentication. Keep test connections on a trusted machine.

## Review retained debug rendering

Build the viewer and the debug example from the workspace root:

```sh
cargo build --release -p rocketsim_viewer --bin rocketsim-viewer
cargo build --release -p rocketsim_vis --example debug_render
```

Run the review harness in showcase and stress modes:

```sh
python3 tools/debug-render-review.py --only showcase --only stress \
  --viewer-arg=--no-menu
```

The harness runs `wire`, `solid`, `showcase`, or `stress` mode. `--only MODE`
can be repeated. The wire and solid modes require `--obj PATH` with a local
ZeroTwo model file:

```sh
python3 tools/debug-render-review.py --obj /path/to/ZeroTwo.obj \
  --only wire --only solid --static --viewer-arg=--no-menu
```

The ZeroTwo model is user-local input. The repository does not include the
model or a derived asset because its license is unknown. The procedural
showcase uses original code and no third-party assets. Use `--static` to keep
the selected scene static. Use `--viewer-arg=ARG` to pass a viewer option;
for example, `--viewer-arg=--no-menu` starts with the menu closed. The harness
also requests `--debug-stats PATH` and saves captures and logs under
`target/input-review/debug-render`. It hides the simulation ball by default
so that the ball does not intersect the debug model. Pass `--show-ball` to
include the ball in a capture.

Pass `--drive` to capture debug geometry in a Soccar arena with a drivable
Octane. This mode keeps the ball visible and requires Soccar collision
meshes:

```sh
python3 tools/debug-render-review.py --only showcase --drive --static
```

For interactive driving, run the example directly. Omit `--seconds` to keep
it running. Press `1` to drive or `F` for free camera:

```sh
cargo run --locked --release -p rocketsim_vis --example debug_render -- \
  --drive --mode showcase
```

## Run the headless debug-render benchmark

This optional Linux benchmark measures the retained `DebugGl` renderer with a
surfaceless EGL pbuffer. The normal viewer build does not need EGL development
libraries. Install EGL development libraries only if you build this optional
`debug-bench` feature.

Build and run the cached-geometry benchmark:

```sh
cargo run --locked --release -p rocketsim_viewer \
  --features debug-bench --bin debug-render-bench -- \
  --lines 100000 --points 10000 --instances 5000 \
  --frames 240 --warmup 60 \
  --output target/input-review/debug-render/native-cached.png \
  --json target/input-review/debug-render/native-cached.json
```

Run the same workload with geometry rebuilt each frame. Use different output
paths:

```sh
cargo run --locked --release -p rocketsim_viewer \
  --features debug-bench --bin debug-render-bench -- \
  --lines 100000 --points 10000 --instances 5000 \
  --frames 240 --warmup 60 --rebuild \
  --output target/input-review/debug-render/native-rebuild.png \
  --json target/input-review/debug-render/native-rebuild.json
```

The benchmark uses `gl.finish()` for controlled timing. It is a renderer-only
test. It is not full-viewer FPS or a comparison with RLBot. Both modes use the
same geometry and produce the same pixels. The benchmark checks that all
readback alpha values are 255. A Xephyr capture with llvmpipe does not give
native GPU timings.

One observed run used an AMD Radeon 780M with radeonsi, Mesa 25.2.8, OpenGL
4.6, a 1280x720 pbuffer, and no MSAA. It used 60 warmup frames and 240 measured
frames. The workload contained 100,000 line segments, 10,000 points, 5,000
instances, and 30 draw calls.

| Mode | CPU mean | GPU mean | Completion wall mean | Upload after warmup |
| --- | ---: | ---: | ---: | ---: |
| Cached geometry | 0.026 ms | 0.349 ms | 0.734 ms | 0 bytes |
| Rebuild geometry | 1.474 ms | 0.723 ms | 3.400 ms | 1,468,800,000 bytes total (6.12 MB/frame) |

These values are one machine's result. They are not a performance guarantee.
A separate readback comparison confirmed identical pixels for cached and
rebuilt geometry.

### Check debug depth and transparency

Run the small GPU regression scenes on a surfaceless EGL driver:

```sh
cargo run --locked --release -p rocketsim_viewer \
  --features debug-bench --bin debug-render-bench -- \
  --check-ordering --output target/input-review/debug-render/ordering.png
```

The checks cover mixed opaque and translucent instances, translucent source
order, camera reversal, and depth-tested versus always-visible lines. They
also check framebuffer alpha and OpenGL errors.

## Capture a viewer screenshot

Use `input-review.py --drive` to capture a scene and stop the test processes
afterward. Build the viewer and drive example first:

```sh
cargo build -p rocketsim_viewer --bin rocketsim-viewer
cargo build -p rocketsim_vis --example drive
python3 tools/input-review.py --profile drive --drive \
  --output target/input-review/drive.png --wait 8
```

Open `target/input-review/drive.png` to inspect the result. The helper starts
Xephyr on `:99`, starts the viewer and drive example, then captures the
display. It stops its test processes when it exits.

The helper refuses to use an active X display. Pass `--display` with an unused
display if `:99` is busy. Pass `--port` with an unused port if another viewer
uses the default port, `39201`.

### Capture another X11 application

Use `vis-shot.sh` for a lower-level capture of an empty viewer or another X11
application. This example uses display `:100` to keep `:99` available for the
input-review helper:

```sh
tools/vis-shot.sh -d :100 -w 4 target/empty-viewer.png -- ./target/debug/rocketsim-viewer 39201
```

The script starts the command after `--` on Xephyr, waits, and captures the
display. It does not start a simulation. Use `-d DISPLAY`, `-g GEOMETRY`, and
`-w SECONDS` to change the display, image size, and capture delay.

Unlike `input-review.py`, this script can reuse an active display and leaves
Xephyr running. It stops only the command process it launched. It does not
stop that command's child processes or isolate its settings. Use the
input-review helper for isolated tests with a simulation.

## Build the input test probe

The probe runs a Soccar simulation, receives controls, and writes JSON Lines
records for controls, actions, mutators, and car state. Build it from its
standalone manifest, then generate the sample input profiles:

```sh
cargo build -p rocketsim_viewer --bin rocketsim-viewer
cargo build --manifest-path tools/Cargo.toml \
  --target-dir target/input-review/control-probe/target
python3 tools/input-review-profiles.py
```

The harness expects the probe at
`target/input-review/control-probe/target/debug/input-control-probe`.
Pass `--control-probe` to start it. Use `--mode thevoid` to test TheVoid instead
of Soccar. `--probe-seconds` limits the probe's lifetime to 1–120 seconds.
The harness can stop it sooner when capture finishes.

## Verify a keyboard binding

Run this test after you build the probe and generate the profiles:

```sh
python3 tools/input-review.py --profile keyboard \
  --settings target/input-review/profiles/keyboard.toml \
  --window 'rocketsim viewer' --control-probe --probe-seconds 30 \
  --action key:Escape --action key:1 \
  --action capture:target/input-review/keyboard-before.png \
  --action key:Up:down --action wait:1 \
  --action capture:target/input-review/keyboard-active.png \
  --action key:Up:up --action wait:1
python3 tools/verify-input-review.py \
  target/input-review/keyboard-controls.jsonl \
  --transition throttle --require 'throttle>=0.8' --state-moved
```

The profile binds the Up key to throttle. The helper selects the first car,
then holds and releases the key. The checker requires neutral and active
throttle values and a change in car position. Inspect the screenshots as well
as the checker result.

## Verify a controller axis

Prepare uinput access, then request a virtual controller with `--controller`:

```sh
python3 tools/input-review.py --profile pad-axis \
  --settings target/input-review/profiles/pad-axis.toml \
  --window 'rocketsim viewer' --controller --control-probe \
  --control-log target/input-review/pad-axis-controls.jsonl \
  --action key:Escape --action key:1 \
  --action axis:ABS_X:0 --action wait:0.5 \
  --action capture:target/input-review/pad-neutral.png \
  --action axis:ABS_X:28000 --action wait:0.5 \
  --action capture:target/input-review/pad-steer-right.png \
  --action axis:ABS_X:0 --action wait:0.5
python3 tools/verify-input-review.py \
  target/input-review/pad-axis-controls.jsonl \
  --transition steer --require 'steer>=0.8'
```

The profile binds the left stick X axis to steering. The checker requires a
change from neutral steering to active steering. The simulation log records
controls and car state, not camera position. Inspect screenshots to check
camera movement, menus, and binding capture.

## Settings, logs, and screenshots

`input-review.py` stores test artifacts under `target/input-review`. It gives
each profile a separate configuration directory. It does not use your normal
viewer settings. The viewer, scene, and Xephyr logs retain earlier runs. Each
probe run replaces its JSON Lines log. Use a different `--control-log` path
when you need to keep earlier records.

Use `--settings` to copy a TOML file into a profile's configuration directory.
To test saved bindings on a later run, use the same `--profile` without
`--settings`. This keeps the saved settings instead of replacing them.

Screenshots and control logs must stay under `target/input-review`. Use one
profile per input test. The generated profiles cover keyboard, mouse buttons,
mouse motion, wheel, controller buttons, axes, triggers, raw codes, and shared
bindings.

## Input actions

Pass actions with repeated `--action` options:

| Action | Result |
| --- | --- |
| `key:KEY` | Press and release an X11 key. |
| `key:KEY:down` / `key:KEY:up` | Hold or release a key. |
| `mouse:N:down` / `mouse:N:up` | Hold or release an X11 mouse button. |
| `click:X:Y[:button]` | Click at a screen position. |
| `move:X:Y` | Move the pointer. |
| `drag:X1:Y1:X2:Y2` | Drag with the primary button. |
| `wheel:UP` / `wheel:DOWN` | Send a scroll event. |
| `pad:BTN_SOUTH[:0|1]` | Release or press a virtual controller button. |
| `axis:ABS_X:VALUE` | Set a virtual controller axis value. |
| `focus:out` / `focus:in` | Remove or restore viewer focus. |
| `wait:SECONDS` | Wait for up to five seconds. |
| `capture:FILE` | Save a screenshot at this point. |

Controller actions require `--controller`. Key names use X11 keysyms, such as
`1` or `Escape`. Controller names use `evdev` codes. Use `--window` with a title
fragment to focus the viewer before sending input.

The checker supports `--neutral`, `--transition FIELD`, `--state-moved`, and
conditions such as `--require 'jump=true'`. Use screenshots for visual results;
the checker does not inspect images.

The helper can start the drive example instead of the probe with `--drive`.
Build the drive example first. Do not combine `--drive` and `--control-probe`.

## Troubleshooting

- **Xephyr does not start:** Check your desktop connection and the selected
  display. Input-review errors appear in `target/input-review/xephyr.log`.
  For `vis-shot.sh -d :100`, check `/tmp/xephyr-100.log`.
- **The port is in use:** Pass `--port` with an unused port. Do not stop another
  user's viewer to free the port.
- **The scene is empty:** Check the simulation's mesh setup and connection.
  Screenshot helpers do not provide collision meshes.
- **Input has no effect:** Use `--window` to focus the viewer. Close settings
  and select a car. Confirm that the profile contains the expected binding.
- **The virtual controller cannot start:** Install Python `evdev` and configure
  permission to create a uinput device. Do not disable host input devices.
- **A connection reports a version mismatch:** Rebuild the viewer and probe
  from the same checkout.

See `python3 tools/input-review.py --help` and
`python3 tools/verify-input-review.py --help` for all command options.
