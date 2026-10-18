# rocketsim_test

Use this crate to test RocketSim changes, compare simulation results with
recorded Rocket League play, and measure simulation speed. It is a workspace
test crate. It is not published to crates.io and is not required by simulation
applications.

## Before you start


- Prepare `collision_meshes/` in the workspace root. See the
  [mesh setup instructions](../README.md#prepare-the-collision-meshes).
- Run the commands below from the workspace root.
- Use a release build for speed measurements. Debug builds are not suitable
  for comparisons with release-build results.

The repository includes short test recordings under
[`tests/rl_comparison_test/test_recordings`](tests/rl_comparison_test/test_recordings)
and longer match recordings under [`recordings`](recordings). The recording
loader supports `.rlpr` files and Zstandard-compressed `.rlpr.zst` files.

## Run the tests

Run this crate's tests and example tests:

```sh
cargo test --locked -p rocketsim_test --all-targets -- --test-threads=1
```

Run the workspace checks before you submit a change:

```sh
cargo test --locked --workspace --all-targets -- --test-threads=1
cargo test --locked --workspace --doc
```

These commands use the Rust simulator. To include the optional C++ RocketSim
comparison implementation, enable `v2`:

```sh
cargo test --locked -p rocketsim_test --all-targets --features v2 -- --test-threads=1
```

The `v2` feature requires a C++ build toolchain. Run its tests serially because
the comparison implementation uses process-global state. See the
[CI guide](../tools/ci/README.md) for all-feature release checks.

## Compare simulation results with recordings

Run the replay metric against the available bundled match recordings:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- --per-recording
```

The metric restores recorded state at the start of each segment. It then
applies recorded inputs and compares the simulated state with later recorded
states. The default segment length is 120 ticks. Use `--segment-ticks` to
change it.

To select one recording, pass its path after `--`:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- \
  rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
```

The report groups results by contact type. `Support` counts car-ticks: one car
compared for one tick counts as one sample. Contact groups can overlap. A
sample passes when its normalized error is less than `1.0`. Lower `Mean norm`
and `Max norm` values indicate less error. `First fail` identifies the first
failed tick in a single-recording report.

The metric prints its tolerances. Compare results with the same recordings,
segment length, and reset mode. A recording can omit simulation state, such
as boost-pad cooldowns. A failed sample does not identify the cause by itself.
The metric reports results; it does not require every sample to pass before
it exits successfully.

### Inspect one-tick errors

Use `--reset-each-tick` to restore recorded state before each scored tick:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- --reset-each-tick
```

Use `--reset-motion-each-tick` to restore only position, linear velocity, and
angular velocity for each car and the ball:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- \
  --reset-motion-each-tick --per-recording
```

The motion-only mode keeps simulated orientation, flags, timers, and boost
between ticks. Each segment still starts with recorded state. State setters
clear transient accumulators and the car impulse cache. They do not
explicitly clear wheel or contact-manifold caches. Use this mode to examine
state-history errors, not as a complete recorded-state reset.

The two reset flags cannot be used together. Both use the one-tick scoring
tolerances. Without either flag, the metric uses segmented comparisons.

To print results for both the Rust and C++ implementations, use:

```sh
cargo run --release -p rocketsim_test --features v2 --example rlpr_metric
```

## Measure simulation speed

Run the default six-car workload:

```sh
cargo run --release -p rocketsim_test --example stress_v3
```

The tool reports elapsed time, ticks per second (`TPS`), and ball hits. Use
`--num-cars` and `--game-mode` to select a workload. Use `--num-arenas` to step
several arenas in the same run. This option does not create worker threads.

Compare memory modes with the same workload:

```sh
cargo run --release -p rocketsim_test --example stress_v3 -- --mem-weight-mode heavy
cargo run --release -p rocketsim_test --example stress_v3 -- --mem-weight-mode balanced
cargo run --release -p rocketsim_test --example stress_v3 -- --mem-weight-mode light
```

For a ball-only workload, pass `--num-cars 0`. This workload runs five times
as many episodes as the normal workload.

Use `stress_v2` with the `v2` feature to measure the C++ implementation:

```sh
cargo run --release -p rocketsim_test --features v2 --example stress_v2
```

### Measure recorded match input

Use the same 3v3 recording for a repeatable comparison:

```sh
cargo run --release -p rocketsim_test --example stress_v3 -- \
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
cargo run --release -p rocketsim_test --features v2 --example stress_v2 -- \
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
```

Replay speed measurements require six cars and Soccar. `--rlpr-file` selects
replay mode instead of the generated-input workload. The tool decodes the
recording and prepares controls before timing starts. The timed loop applies
controls, steps the simulation, and restores state at discontinuities such as
kickoffs and demolition respawns. It does not collect accuracy metrics or
inspect events.

Run each speed measurement more than once on the same machine. Keep build
options and workload settings the same. Do not compare the replay metric's
runtime with the stress tool's runtime; they measure different work.

## Troubleshooting

- **Mesh initialization fails:** Check the workspace `collision_meshes/`
  directory. Provide complete, compatible mesh sets before you run the tests.
- **The C++ build fails:** Install a C++ build toolchain, or omit `--features v2`
  if you only need the Rust implementation.
- **All-feature tests fail only in parallel:** Rerun with `--test-threads=1`.
- **A recording cannot be loaded:** Check its path and file format. Use
  `rlpr_metric --help` through `cargo run` for the supported command options.
- **A measurement differs from the README:** Check the machine, build profile,
  memory mode, number of cars, and workload before you compare results.
