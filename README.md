![A logo saying RocketSim](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

**A Rust library for simulating Rocket League games at maximum efficiency**

## What is this?

RocketSim v3 — a Rust library for simulating Rocket League games at maximum efficiency.

What you get:

- Full car + ball physics: driving, jumping/flips, boost, demos/bumps, world + car-ball collisions
- All game modes: Soccar, Hoops, Dropshot, Snowday, Heatseeker, plus `TheVoid` for testing
- Configurable mutators, arenas, boost pads, ball options, and deterministic RNG seeds
- Tunable memory/performance tradeoff via `ArenaMemWeightMode` (`Heavy`/`Balanced`/`Light`)
- Validated by replay: segment accuracy metric and throughput benchmarks on a bundled 3v3 recording

Built for bots, RL training, and large-scale simulation where you need thousands of arenas ticking in parallel.

## Why v3?

v3 is the faster, more accurate RocketSim.

- ~4-5x faster than v2 in both random-bot and real-replay throughput.
- More accurate than v2 on real Rocket League play — both over full open-loop segments and on isolated single-tick error.
- Pure Rust, simpler to build and embed, no C++ dependency.
- Tunable memory: `Heavy` for max speed, `Balanced` for much less memory at near-`Heavy` speed, `Light` for minimal footprint.
- Deterministic and configurable: seeded RNG, custom arenas/boost pads/mutators, all game modes.

## Benchmarks

Numbers move as development continues, so only the rough gap is stated. Run these yourself:

### Arena memory modes

`ArenaConfig::mem_weight_mode` controls the broadphase memory/performance tradeoff. In the `stress_v3` benchmark:

| Mode              | 1v1 memory per arena | 1v1 performance | 3v3 memory per arena | 3v3 performance |
| ----------------- | -------------------: | --------------: | -------------------: | --------------: |
| `Heavy` (default) |             ~915 KiB |        Baseline |             ~946 KiB |        Baseline |
| `Balanced`        |             ~108 KiB |    ~1–2% slower |             ~123 KiB |      ~3% slower |
| `Light`           |              ~66 KiB |      ~9% slower |              ~91 KiB |     ~12% slower |

Balanced reduces marginal memory while retaining most of Heavy's performance. Light uses one broadphase cell to reduce memory use. Results can differ by platform and workload.

### Accuracy and throughput

Run the v3 metric:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric
```

Enable `v2` to compare v3 with the original C++ RocketSim bindings:

```sh
cargo run --release -p rocketsim_test --features v2 --example rlpr_metric
```

Run the comparable stress benchmarks with these commands:

```sh
cargo run --release -p rocketsim_test --example stress_v3
cargo run --release -p rocketsim_test --features v2 --example stress_v2
```

For a ball-only workload (no cars), pass `-n 0`. Ball-only runs use five times
as many episodes as the normal stress workload:

```sh
cargo run --release -p rocketsim_test --example stress_v3 -- --num-cars 0
cargo run --release -p rocketsim_test --features v2 --example stress_v2 -- --num-cars 0
```

Replay mode takes precedence: `--rlpr-file` always runs replay validation.

Replay the bundled 3v3 recording for a deterministic throughput comparison:

```sh
cargo run --release -p rocketsim_test --example stress_v3 -- \\
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
cargo run --release -p rocketsim_test --features v2 --example stress_v2 -- \\
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
```

Replay mode requires six cars and Soccar. It decodes the recording and
prepares all controls before timing starts. The timed loop applies controls,
steps the simulation, and restores state only at discontinuities such as
kickoff or demo respawn. It does not collect metrics or inspect events.
