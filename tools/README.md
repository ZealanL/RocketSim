# vis screenshot harness

`tools/vis-shot.sh` captures a nested Xephyr display to PNG for human review.
Every visual change is verified with a screenshot — nothing lands without eyes on it.

## Prerequisites

Xephyr, xwd (`x11-apps`), Python 3 + PIL + numpy. All already present on the
dev box. The converter is `tools/xwd2png.py` (TrueColor ZPixmap, vectorized).

## Usage

```sh
# prove the harness with a trivial client (leaves Xephyr :99 running):
tools/vis-shot.sh /tmp/probe.png -- xterm -e 'sleep 5'

# build the viewer first, then capture it (the viewer listens on 39201):
cargo build -p rocketsim_viewer --bin rocketsim-viewer
tools/vis-shot.sh -w 8 /tmp/shot.png -- ./target/debug/rocketsim-viewer 39201

# flags: -d DISPLAY (default :99), -g GEOM (default 1280x720), -w WAIT secs
```

The viewer binds to `127.0.0.1` by default. Do not expose it on an untrusted
network. Its protocol has no authentication.

The script ensures Xephyr is up, runs the command with `WAYLAND_DISPLAY`
unset, waits, captures root, converts, then stops the command if still alive.

## Deterministic scene

`cargo run -p rocketsim_viewer --example photo` steps a fixed seed-0 Soccar
scene (blue drives, orange idles) 600 ticks and prints ball/car0 positions.
Build `rocketsim-viewer` first or install it on `PATH` so the example can
connect. The viewer renders scene mesh metadata from the stream. The simulation
still initializes its own meshes. Same seed + same ticks = same output;
screenshot timing may vary.

## Perf log (running table, CPU submit ms + wall fps)

| scene | cpu ms | wall fps | notes |
|-------|--------|----------|-------|
| harness probe (xterm) | n/a | n/a | pipeline proof (xclock shot) |
| empty window + overlay | ~0 (idle, repaint-on-demand; 4 frames/5s) | n/a (no loop) | — |
| wire protocol (headless) | update ~1µs mean (gate <100µs); encode ~50µs @6car (gate <200µs) | n/a | payloads: 0car 676B / 1car 942B / 2car 1208B / 6car 2272B |
| triangle smoke test | draw 0.03ms CPU; 61fps wall (cap 60) | 61 | — |
| arena mesh | draw 0.87ms CPU; 62fps wall (cap 60) | 62 | weld 5.27x soccar (24090→4570), 5.60x hoops, 5.41x dropshot |
| textured ball + cars | scene draw ~1.9ms CPU (llvmpipe; watch); 62fps wall | 62 | white ball + blue/orange customs + team grid, live tick 1163 |
| pads, instanced | scene draw ~1.9ms CPU (llvmpipe); 62fps wall | 62 | 4 instanced draws, 29/34 + 5 culled live, pill close-up verified |
| lines and trails | scene draw ~2.2ms CPU (llvmpipe); 62fps wall | 62 | 37 segs, ground line crisp, trail + boost via action sim, zero rainbow |
| camera math | headless (camera-math suite) | n/a | rigid yaw, freezes, swing, FOV, director holds, auto rules |
| birds-eye + car-cam | scene draw ~2.0ms CPU (llvmpipe); 62fps wall | 62 | RL ball-cam verified, frustum follows (16/18 culled in follow) |
| director + auto | scene draw ~1.5-1.7ms CPU (llvmpipe); 62fps wall | 62 | threat cut to car + boost flame, auto ball-cam chase; retreat rule also observed live |
| menus + settings | scene draw ~1.8ms CPU (llvmpipe); 60fps wall | 60 | Escape menu live, TOML persist + toggle proof (pads off/FOV 90 loaded) |
| effects | scene draw ~2ms CPU (llvmpipe); 60fps wall | 60 | dust/ring/flash/landing shot, glow unit-pinned; trigger is acquisition+cooldown |
| HW | real-GPU numbers (:0, 1280x720) | cpu ~0.08ms, submit ~0.01ms, gpu-work ~1.0ms (stall), present/egui ~0.5ms; ~1500fps uncapped, 167fps vsync-locked | — | vsync=false proven live; no code action justified (26x headroom @60fps/1440p) |
| HW2 | A/B attribution (follow0, uncapped) | lines 0.30→0.02ms via tight quads; MSAA4 = 0.20ms (28%); pads ~0.3ms | — | tight-quad optimization landed (10x line-cost cut, vis-diff clean); MSAA is now the top lever (user knob: try 0/2) |
| HW3 | draw-call batching | 17→10 draws/frame (instanced cars, merged arena, empty skips); ~20 fewer driver calls, single-digit µs real-HW est. | — | pixel-identical (0.000% vs pre-batch binary); llvmpipe can't resolve the delta |

Budget: <1ms CPU submit/frame, 60fps cap. Numbers recorded here.

## Input review harness

Run the input review against a stable viewer build. The harness uses the isolated
`target/input-review` tree. It does not use the user's normal config. It starts
Xephyr on `:99`, launches each child with its own `XDG_CONFIG_HOME`, and stops
the child processes when it exits.

The `input-control-probe` runs a real Soccar or TheVoid arena. It receives
`ControlMsg` values from the viewer over the normal sim connection. It applies
the controls and writes JSON Lines records for controls, actions, mutators, and
car state. This verifies the viewer-to-simulation path. Build the probe and
viewer from the same checkout. Version 0.3 uses wire protocol 3; older viewer
binaries cannot read its car snapshots.

Build after the viewer source is stable. The probe manifest and source are tracked
in `tools/Cargo.toml` and `tools/input-control-probe.rs`. Cargo keeps the probe's
dependency lockfile in `tools/Cargo.lock`. Use a target directory under ignored
`target/input-review` for build files.

```sh
cargo build -p rocketsim_viewer --bin rocketsim-viewer
cargo build -p rocketsim_vis --example drive
cargo build --offline --manifest-path tools/Cargo.toml --target-dir target/input-review/control-probe/target
python3 tools/input-review-profiles.py
```

Create source-specific profiles under the ignored target tree. Start one run per
profile so each run has a clean simulator log. Example: test the keyboard throttle
binding and save before/active/after screenshots:

```sh
python3 tools/input-review.py --profile keyboard \
  --settings target/input-review/profiles/keyboard.toml \
  --window 'rocketsim viewer' --control-probe --probe-seconds 30 \
  --control-log target/input-review/keyboard-controls.jsonl \
  --action key:Escape --action key:1 \
  --action capture:target/input-review/keyboard-before.png \
  --action key:Up:down --action wait:1 \
  --action capture:target/input-review/keyboard-active.png \
  --action key:Up:up --action wait:1
python3 tools/verify-input-review.py \
  target/input-review/keyboard-controls.jsonl \
  --transition throttle --require 'throttle>=0.8' --state-moved
```

Use `--controller` to create the uinput pad before the viewer starts. The helper
supports buttons, signed sticks, triggers, and hats. Example: verify that a pad
stick reaches the sim as steering:

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

The profile stores the follow binding as `Num1`. Send X11 keysym `1` with the
harness to select Follow 1. Use `--mode thevoid` to run the probe in TheVoid.
The default mode is Soccar.
The `--drive` option starts the stock drive example instead of the JSON probe.
Do not pass `--drive` and `--control-probe` in the same run.

The helper accepts these action types:

- `key:KEY`, `key:KEY:down`, and `key:KEY:up` send key events. A short key press
  stays down for 150 ms.
- `mouse:N:down` and `mouse:N:up` hold or release an X11 mouse button.
- `move:X:Y` moves the mouse to an X11 screen coordinate. Use it to test mouse
  look. Capture before and after images in free-camera mode.
- `drag:X1:Y1:X2:Y2` holds mouse button 1 while it moves between the points.
  Use it to resize the settings window.
- `wheel:UP` and `wheel:DOWN` send one scroll event.
- `pad:BTN_SOUTH[:0|1]` presses or releases an evdev button.
- `axis:ABS_X:VALUE` sends an evdev axis value. Other registered `ABS_*` axis
  names work in the same form.
- `focus:out` moves focus away from the viewer. `focus:in` restores it.
- `wait:SECONDS` waits for up to 5 seconds. Use this for control and save debounce.
- `capture:FILE` writes a screenshot at that point in the action list.

The harness writes each profile to `target/input-review/<profile>-controls.jsonl`
by default. Set `--control-log` to choose another path under `target/input-review`.
Pass that path as the first argument to the checker.

The JSON checker accepts dotted control names. For example, use
`--require 'jump=true'`, `--action-bit 0`, `--button-bit 0`, or
`--mutator-bit 1` to check jump, reset, a control pulse, or a mutator. Use
`--neutral` to require a neutral control sample. Use `--transition throttle`
to require both neutral and active throttle values. Use `--state-moved` to require a car position change.

The current viewer has 56 actions and seven input source types: key, mouse
button, mouse axis, wheel, pad button, pad axis, and raw pad code. The unit test
`exhaustive_sources_actions_and_slots_roundtrip_and_evaluate_edges` checks every
concrete source in both binding slots for every action. This includes at least
392 action/source-type combinations. The test also checks the source variants
for keys, buttons, axes, and raw codes. Run it after the source is stable:

```sh
cargo test -p rocketsim_viewer input::tests::exhaustive_sources_actions_and_slots_roundtrip_and_evaluate_edges
```

The real-window review uses representative end-to-end profiles for each source
type:

| Profile | Input | Required evidence |
|---|---|---|
| `keyboard` | Up/Down keys | throttle changes in the sim control log |
| `mouse-button` | primary/secondary buttons | jump or boost changes in the sim control log |
| `mouse-axis` | signed mouse motion | free-camera screenshot changes in the expected direction |
| `wheel` | scroll up/down | throttle changes in the sim control log |
| `pad-button` | South/East buttons | jump or boost changes in the sim control log |
| `pad-axis` | left-stick X | steer changes in the sim control log |
| `pad-trigger` | trigger axes | free-camera height changes in the screenshot |
| `pad-raw` | raw ABS_X code (`196608`) | throttle changes in the sim log |
| `shared-key` | one key bound to throttle and jump | both control fields change in one run |

Capture the single Bindings section. Rebind one action. Capture the armed state
and the resulting binding. Then check the TOML file in
`target/input-review/config/<profile>/rocketsim-vis/settings.toml`. On the
restart run, pass `--profile <profile>` but do not pass `--settings` again. This
keeps the saved file. Repeat the input and check the control log. Also capture
free-camera before and after keyboard movement, mouse motion, and pad-stick
motion. Use screenshots for view actions; the sim log does not contain camera
poses.

The viewer selects the Xbox-style virtual pad for these runs. Require matching
control records from `input-control-probe` for pad driving tests. Do not disable
or grab the host input devices.

For focus-loss review, hold a bound input, use `focus:out`, wait, and capture.
Restore focus while the input is still held. Confirm that the sim receives
neutral controls until the input is released. Repeat with a pad axis and a pad
button. For shared bindings, assign the same source to two actions and assert
both outputs. Require a control or action record from `input-control-probe` for
driving and simulation actions. Require a camera or menu screenshot change for
view-only actions.
