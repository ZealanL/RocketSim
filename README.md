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
