# RocketSim project instructions

These instructions apply to the full repository. Read the relevant crate guide
before you change its behavior. Preserve work that is already in the worktree.
Do not stage changes, create branches, or commit unless the user requests it.

## Project structure

This is a Rust 2024 workspace. The minimum Rust version is set in `Cargo.toml`.

- `rocketsim/`: Core physics library. It must work without a window or graphics
  dependencies.
- `rocketsim_test/`: Physics regression tests, recorded-play comparisons, and
  simulation benchmarks. Its optional `v2` feature uses the C++ implementation.
- `rocketsim_vis/`: Simulation attachment, retained debug drawings, protocol,
  and network transport. Do not add viewer graphics dependencies here.
- `rocketsim_viewer/`: Separate OpenGL viewer, cameras, input, and settings.
- `tools/`: Screenshot review, input integration, and CI tools.
- `target/`: Build output and local validation artifacts. Do not commit them.

Keep these boundaries:

- In `rocketsim_vis/src/`, `arena.rs` integrates the simulation. `debug/` owns
  drawing types and retained storage. `proto/` owns messages and codecs.
  `sync/` owns full and delta scene updates. `transport/` owns socket I/O.
- In `rocketsim_viewer/src/`, `app/` coordinates the application. `ui/` owns
  panels. `geometry/` owns CPU geometry. `render/` owns GPU resources and draw
  passes. `effects/`, `input/`, `camera/`, and `settings.rs` own their named
  responsibilities.
- Keep `main.rs` small. Do not move application state back into the entry point.
- Separate pure transforms and geometry preparation from OpenGL calls.
- Keep implementation modules private unless callers need a public API.
  Keep common drawing imports under `rocketsim_vis::debug`.
- Keep demo fixtures in example support modules, not the public library API.
  Do not include Rust source from a sibling package. Each package must build
  from its own packaged files and declared dependencies.

## Code and documentation

- Use ASD-STE100 Simplified Technical English for documentation, comments,
  and user-facing text. Use short sentences and consistent technical names.
- Document public API behavior, units, defaults, errors, and ownership rules.
  Use runnable rustdoc examples when they help a caller complete a task.
- Use existing dependencies and patterns. Do not add a general framework
  when a direct implementation is sufficient.
- Use `glam::Vec3A` and `glam::Mat3A` for computational 3D math. Keep transport
  and GPU data packed. Do not reinterpret aligned SIMD types as packed arrays.
- Reuse buffers and immutable `Arc` snapshots in repeated work. Avoid deep
  frame clones and unchanged geometry uploads.
- Preserve deterministic behavior. For physics changes, compare results with
  fixed seeds and the relevant bundled recordings.

## Protocol and retained scenes

- Keep `PROTOCOL_VERSION` at `1` until the first release. The current wire
  format is not released. Rebuild the simulation and viewer together after
  a wire change. Update both ends, validation, and documentation.
- Keep graphics out of the simulation update. Network writes and serialization
  must not make physics wait for a viewer.
- Physics frames are latest-only. Retained scene changes must survive skipped
  frames, pauses, and reconnects. Preserve removals and control action pulses.
- Interleave debug chunks with physics frames. Neither stream may starve.
- The simulation-message cap is 32 MiB. Debug chunks are 16 KiB. The first
  frame can contain arena metadata larger than 64 KiB. Do not restore a
  64 KiB cap on simulation messages.
- Keep retained byte estimates conservative. Include normals, UVs, identifiers,
  and message overhead. Never mark a failed transfer as sent.
- Use one `ViewerSnapshot` per viewer update. Use its frame, scene, debug
  scene, and generation for rendering, projected labels, and control requests.
- Preserve canonical retained state when filtering expired drawings for display.
  `Lifetime::Frame` is tied to simulation updates, not a wall-clock timeout.
- Do not restore the removed `FrameClient`, per-frame `DebugDraw`, standalone
  frame codecs, or separate control connection without an explicit requirement.

## Rendering constraints

- Opaque world geometry writes depth. Keep translucent instance ordering stable
  and back-to-front. Batch sorted instances only when their order is preserved.
- Current transparency sorting is per instance. Do not claim exact sorting of
  intersecting translucent triangles.
- Preserve framebuffer alpha, Hoops geometry, and debug depth behavior.
- Draw world overlays after world effects. Keep screen overlays distinct from
  depth-tested world geometry. Do not implement screen drawing with geometry
  placed in front of the camera.
- Create and destroy OpenGL resources on the GL context thread.
- Preserve settings-file migration, input focus handling, and release gates
  when changing input code. Runtime input uses the canonical `ActionMap`.

## Test smart, not hard

Start with the affected behavior. Use an existing regression scenario when it
can cover the change. Run broader checks for changes across module boundaries.

- Prefer a small set of meaningful scenarios over many isolated unit tests.
- Combine related cases into table-driven tests. Remove duplicate or vacuous
  tests when their behavior has better coverage elsewhere.
- Do not add tests that only repeat constants, check trivial getters, search
  shader source strings, or retest dependency behavior.
- Keep useful coverage for physics results, expiry, reconnects, partial writes,
  stale generations, focus changes, clipping, and render order.
- Do not remove useful assertions just to reduce the test count.
- Use controlled clocks instead of sleeps for pure lifetime tests. Bound waits
  when testing real threads or sockets.
- For physics changes, use the checks in `rocketsim_test/README.md`. Report
  changes in recorded-play results. Do not change tolerances just to pass.

Run commands from the workspace root. Select the affected package and test
filter first. For changes across the visualization crates, use:

```sh
cargo test --locked -p rocketsim_vis -p rocketsim_viewer \
  --all-targets --all-features -- --test-threads=1
cargo test --locked -p rocketsim_vis -p rocketsim_viewer --doc --all-features
```

Use these formatting and lint checks when relevant:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
```

Use nightly rustfmt when the nightly toolchain is available. Do not use
`cargo fmt`; it can select stable rustfmt and ignore nightly-only settings.
If nightly is unavailable, report that the formatting check could not run.

Workspace all-feature checks enable the optional C++ comparison code. They need
its toolchain. Do not make C++ a requirement for normal Rust-only use.

Keep the visualization API documentation warning-free:

```sh
RUSTDOCFLAGS='-D warnings -D missing_docs' \
  cargo doc --locked -p rocketsim_vis --no-deps --all-features
```

## Visual and performance validation

Use `tools/README.md` for the current screenshot and input workflows.

- For visible changes, capture the affected scene and inspect the saved image.
  A successful launch or a screenshot file alone does not prove correctness.
- Use the isolated Xephyr harness when testing the X11 viewer. Do not use
  X11 tools to control native Wayland windows. Do not alter host input-device
  permissions or stop processes that the test did not start.
- Store screenshots, logs, and benchmark output under `target/input-review/`.
- For input changes, check both received controls and resulting car movement.
  Inspect captures for camera, menu, and binding behavior.
- For depth, transparency, built-in assets, or GPU cache changes, use the
  existing `debug-render-bench --check-ordering` scenarios.
- Measure performance in release builds. Record the workload, renderer,
  warmup, measured frames, and CPU/GPU timing method.
- Compare the same work and output. Record draw calls and upload bytes as well
  as timings. If the benchmark scope changes, state that results are not
  directly comparable.
- Xephyr with llvmpipe is suitable for visual review, not native GPU timing.
  Use the optional `debug-bench` feature and native EGL for GPU measurements.
- Do not report renderer-only timing as full-viewer FPS. Do not claim a speed
  improvement from a single noisy run.

## Assets and local setup

- Collision meshes are external assets. Do not add acquisition instructions
  or redistribute them without verified permission.
- Mesh initialization succeeds once per process. Load all needed modes on the
  first successful call. `TheVoid` supports checks without collision meshes.
- The Zero Two OBJ is user-local input. Its redistribution license is not
  verified. Do not commit the OBJ or derived assets.
- Check licenses before adding other assets. Preserve the bundled viewer model
  terms in `rocketsim_viewer/src/models/MODEL_LICENSE_README.md`.
