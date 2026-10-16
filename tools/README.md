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

# capture the viewer (viewer runs its own main loop):
tools/vis-shot.sh -w 8 /tmp/shot.png -- ./target/debug/rocketsim-viewer 39201

# flags: -d DISPLAY (default :99), -g GEOM (default 1280x720), -w WAIT secs
```

The script ensures Xephyr is up, runs the command with `WAYLAND_DISPLAY`
unset, waits, captures root, converts, then stops the command if still alive.

## Deterministic scene

`cargo run -p rocketsim_viewer --example photo` steps a fixed seed-0 Soccar
scene (blue drives, orange idles) 600 ticks and prints ball/car0 positions.
Same seed + same ticks = same output; screenshot timing may vary, sim must not.

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
