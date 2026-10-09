# vis screenshot harness

`tools/vis-shot.sh` captures a nested Xephyr display to PNG for human review.
Every 3D stage is gated on a screenshot — no stage lands without eyes on it.

## Prerequisites

Xephyr, xwd (`x11-apps`), Python 3 + PIL + numpy. All already present on the
dev box. The converter is `tools/xwd2png.py` (TrueColor ZPixmap, vectorized).

## Usage

```sh
# prove the harness with a trivial client (leaves Xephyr :99 running):
tools/vis-shot.sh /tmp/probe.png -- xterm -e 'sleep 5'

# stage 1+: capture the viewer (viewer runs its own main loop):
tools/vis-shot.sh -w 8 /tmp/shot.png -- ./target/debug/vis_viewer 39201

# flags: -d DISPLAY (default :99), -g GEOM (default 1280x720), -w WAIT secs
```

The script ensures Xephyr is up, runs the command with `WAYLAND_DISPLAY`
unset, waits, captures root, converts, then stops the command if still alive.

## Deterministic scene

`cargo run -p rocketsim_vis --example photo` steps a fixed seed-0 Soccar
scene (blue drives, orange idles) 600 ticks and prints ball/car0 positions.
Same seed + same ticks = same output; screenshot timing may vary, sim must not.

## Perf log (running table, CPU submit ms + wall fps)

| stage | scene | cpu ms | wall fps | notes |
|-------|-------|--------|----------|-------|
| 0 | harness probe (xterm) | n/a | n/a | pipeline proof (xclock shot) |
| 1 | empty window + overlay | ~0 (idle, repaint-on-demand; 4 frames/5s) | n/a (no loop) | ref `rocketsim_vis/tests/reference/stage1_empty.png` |
| 2 | wire protocol (headless) | update ~1µs mean (gate <100µs); encode ~50µs @6car (gate <200µs) | n/a | payloads: 0car 676B / 1car 942B / 2car 1208B / 6car 2272B |
| 3 | first triangle | draw 0.03ms CPU; 61fps wall (cap 60) | 61 | ref `rocketsim_vis/tests/reference/stage3_triangle.png` |
| 4 | arena mesh | draw 0.87ms CPU; 62fps wall (cap 60) | 62 | ref `rocketsim_vis/tests/reference/stage4_arena.png`; weld 5.27x soccar (24090→4570), 5.60x hoops, 5.41x dropshot |
| 5 | ball + cars | | | |
| 6 | pads instanced | | | |
| 7 | lines/trails | | | |
| 8a-d | cameras | | | |
| 9 | menus + settings | | | |
| 10 | effects | | | |

Budget: <1ms CPU submit/frame, 60fps cap. Numbers recorded per stage here.
