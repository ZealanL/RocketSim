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

## Code layout

- `src/rlpr/` decodes recorded bytes and field availability.
- `src/replay/` owns shared recorded-state restore policy. It uses the public
  simulator body and contact APIs.
- `examples/rlpr_metric/common/` owns segmentation, scoring, history gates,
  field coverage, and transition reports.
- `examples/rlpr_metric/v3/` maps supported recorded state to the Rust backend.
- `tests/fixtures/rlpr/` holds small synthetic outputs from the C++ recorder.
  These files verify wire compatibility. They contain no game data. The
  generator sources stay in `rl_phys_record`, not this crate.

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
the comparison implementation uses process-global state. Workspace CI skips
`v2` because `rocketsim-rs` 0.37.0 fails to compile with GCC 11. Use GCC 13 to
test this feature.

## Compare simulation results with recordings

Run the replay metric against the available bundled match recordings:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- --per-recording
```

The metric creates a fresh arena for each recording. It restores supported
recorded state at the start of each segment. It then applies recorded inputs
and compares the simulated state with later recorded states. The default
segment length is 120 ticks. Use `--segment-ticks` to change it.

To select one recording, pass its path after `--`:

```sh
cargo run --release -p rocketsim_test --example rlpr_metric -- \
  rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst
```

The bundled files use RLPR v12. Each file has adjacent
`<recording filename>.metadata.json` roster metadata. The metric checks the
SHA-256 of the exact input bytes before it uses the metadata. The metadata
contains a measured body and team for every frozen car slot. A stale hash,
unknown name, or wrong roster length is an error.

Use `--car-bodies` and `--car-teams` to override the metadata. This example
specifies the measured Daizen roster:

```sh
cargo run --locked --release -p rocketsim_test --example rlpr_metric -- \
  --car-bodies plank,plank,plank,plank --car-teams blue,orange,orange,blue \
  rocketsim_test/recordings/daizen_2v2_300s.rlpr.zst
```

Names are case-insensitive. Each list must contain one entry per car slot in
every selected recording. CLI values take precedence over metadata. Without
metadata, the metric uses the header and settled wheel height for bodies.
Octane and Hybrid share wheel geometry and cannot be separated this way.
The header describes car 0 only. Teams fall back to alternating Blue/Orange
by frozen slot. The metric warns when it uses an unverified fallback.

See the [corpus guide](recordings/README.md) for capture identities, rosters,
and remaining state gaps.

The report groups results by contact type. `Support` counts car-ticks: one car
compared for one tick counts as one sample. Contact groups can overlap.
`no_contact` means no labeled contact anywhere in the tick, including the
shared ball. A sample passes when its normalized error is less than `1.0`.
Lower `Mean norm` and `Max norm` values indicate less error. `First fail`
identifies the first failed tick in a single-recording report.

The metric prints each file's RLPR version and SHA-256. It validates available
body state and controls before segmentation. Invalid recorded state is an error,
not an excluded sample. The error identifies the row, native frame, and car slot.
Non-finite simulated errors remain failed samples with infinite error. The
metric reports their count and exits with an error.

The metric prints its tolerances, availability, excluded transition reasons,
skipped car-ticks, and restore coverage. Transition counts use deduplicated
rows and the actual replay segments. Each incoming transition is scored,
stepped as countdown stasis, or excluded with one primary reason. A random
kickoff placement or demolition respawn is a reset boundary. The first
subsequent deterministic step stays scored. A direct countdown release is
re-seeded from the last countdown row before its live step is scored.

V12 restores supported body internals and supersonic state. The core recorded
wheel `has_contact` and `is_on_ground` remain authoritative. A world-only wheel
flag is not a substitute for either field. The v12 world-contact and jump
component groups are reported as captured but unused. A valid recording prefix
also reconstructs auto-flip and airborne clocks. Raw pad-counter transitions
reconstruct supported pad cooldowns. Missing history stays partial. These
cooldowns are derived values, not captured native timers.

The v3 metric restores retained car–ball and car–car contact geometry.
`src/replay/collision.rs` owns recorded-slot mapping, contact coefficients,
coverage diagnostics, and the rule for clearing missing captured pairs. The
simulator exposes only general body and contact operations through `physics`.
The adapter replaces points in place and preserves existing manifold indices.
It does not restore world contacts, broadphase order, or uncaptured warmstart
impulses. The report separates complete field mappings from partial groups
and counts the writes made at resets. Inferred bump cooldowns use only events
at or before the state row within the same continuous run. They never use the
target outcome to clear a gate.

Captured ball activation and deactivation time restore through `BallState`.
Native tensors, interpolation, and sleeping thresholds use optional body
overrides after the normal state setters. Captured ball sleep remains applied.
Captured-build wake parity is not proven.
The experimental upstream active-island wake rule regresses the external corpus
and contradicts the cached native build. It is not applied. Removing valid sleep
can raise a score on one corpus and lower it on another. It is not a test
correction.

Compare results with the same recordings, segment length, and reset mode.
A failed sample does not identify the cause by itself. The metric reports
results; it does not require every sample to pass before it exits
successfully.

### Compare with the RocketGPU full-state suite

The external `RocketGPU-fullstate-suite` starts RocketGPU from full internal
checkpoints. Its v3 worker starts from public state. It also supplies recorded
boost credits and render-frame inputs to the full-state backend. These results
are not interchangeable with partial-state RLPR replay.

The 2026-11-03 audit reproduced all 12,038,088 exact body-ticks with RocketGPU
`ed2d5e7` on its CPU backend. A public-state ablation passed 96.718% of that
suite's 9,441,924 car-ticks. The corrected released-v3 adapter passed
86.255–86.256%. Those percentages use the external corpus and segmented budgets.
They do not measure reset-mode accuracy on the bundled RLPR files.

The external release does not include recorder source or game-build hashes.
Its capture provenance remains unverified. No CUDA validation was available.
The scorer and its AGPL implementation remain in the external repository.

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
between ticks. Each segment starts with supported recorded state. A direct
countdown-to-live release also gets a full recorded seed.
Per-tick motion setters overwrite the seeded body tensors, interpolation,
and sleep state. They clear transient accumulators and the car impulse cache.
They do not explicitly clear wheel or contact-manifold caches. Use this mode
to examine state-history errors, not as a complete recorded-state reset.

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

`stress_v3` uses the simulator default memory mode, `Balanced`. `stress_v2`
keeps `Heavy` as its default because the C++ backend does not support `Balanced`.
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

Use the same 3v3 recording and explicit memory mode for a repeatable comparison:

```sh
cargo run --release -p rocketsim_test --example stress_v3 -- \
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst --mem-weight-mode heavy
cargo run --release -p rocketsim_test --features v2 --example stress_v2 -- \
  --rlpr-file rocketsim_test/recordings/wisp_3v3_300s.rlpr.zst --mem-weight-mode heavy
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
