![RocketSim](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

# RocketSim

RocketSim simulates Rocket League car and ball physics in Rust. Use it to
test bots, train agents, predict ball motion, or run many independent arenas.
The simulation does not start Rocket League or open a window.

It supports Soccar, Hoops, Dropshot, Snowday, and Heatseeker. `TheVoid` provides
an arena without a collision-mesh hull for tests. You can change mutators,
car bodies, boost pads, and random seeds.

## Choose a crate

| Task | Crate |
| --- | --- |
| Run a simulation without a window | [`rocketsim`](rocketsim/README.md) |
| Connect your simulation to a viewer | [`rocketsim_vis`](rocketsim_vis/README.md) |
| View an arena or drive a car | [`rocketsim_viewer`](rocketsim_viewer/README.md) |

The viewer is optional. Simulation applications do not need the viewer's
graphics dependencies.

## Add RocketSim to your application

The core simulation does not need a C++ compiler.

Add this dependency to your `Cargo.toml`:

```toml
[dependencies]
rocketsim = "0.3.0"
```

The optional `serde` feature adds serialization support to simulation types:

```toml
[dependencies]
rocketsim = { version = "0.3.0", features = ["serde"] }
```

Use one dependency entry, not both.

## Prepare the collision meshes

The simulation needs arena collision meshes before you create an arena.
The crates do not include these files. `init_from_default` reads
`collision_meshes/` relative to your process's working directory:

```text
collision_meshes/
  soccar/     16 .cmf files
  hoops/      12 .cmf files
  dropshot/    2 .cmf files
```

Provide the complete mesh set for each mode you need. Snowday and Heatseeker
use the Soccar mesh set. `TheVoid` does not need collision meshes.

Mesh files are external assets. Review their source and license before you
redistribute them.

For another mesh location, call `rocketsim::init` with that directory instead
of `init_from_default`. Use `init_from_mem` to load `.cmf` byte buffers.
Initialization succeeds once per process. You can retry a failed
initialization. After initialization succeeds, later calls do not replace
the loaded meshes. Load all required modes on the first successful call.

## Run your first simulation

Put this code in `src/main.rs`. Run the application from the directory that
contains `collision_meshes/`.

```rust,no_run
use rocketsim::{
    Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default,
};

fn main() -> std::io::Result<()> {
    init_from_default(true)?;

    let mut arena = Arena::new(GameMode::Soccar);
    let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    arena.reset_to_random_kickoff(Some(0));
    arena.set_car_controls(car, CarControls::default().with_throttle(1.0));

    for _ in 0..120 {
        arena.step_tick();
    }

    println!("Car position: {}", arena.get_car_state(car).phys.pos);
    println!("Ball position: {}", arena.get_ball_state().phys.pos);
    Ok(())
}
```

This prints the car and ball positions after one simulated second. It does
not open a window. Each `step_tick` advances the simulation by `1/120` second.
The application controls how many ticks run. Do not add a sleep when you need
the simulation to run as fast as possible.

Controls remain active until you replace them. Send `CarControls::default()`
to release all controls. Release `jump` before you press it again for another
jump or flip. Read collision and boost-pickup events from the slice returned
by `step_tick`.

Positions and linear velocities use Unreal units. There are 50 Unreal units
per meter. `+Z` points up. Blue defends the `-Y` goal; Orange defends the `+Y`
goal. Angular velocities use radians per second.

## See the simulation

From a source checkout with collision meshes prepared, run:

```sh
cargo run --release --example drive
```

This builds the matching viewer, starts a simulation, and opens the viewer.
Select a car with `1` through `8` to drive it. Press `Escape` to open settings.
See the [viewer guide](rocketsim_viewer/README.md) for controls and installation.
See the [simulation connection guide](rocketsim_vis/README.md) to add viewing
to your own application.

## Configure an arena

Use `ArenaConfig` to set mutators, memory use, and the random seed before you
create an arena:

```rust
use rocketsim::{Arena, ArenaConfig, ArenaMemWeightMode, GameMode};

let config = ArenaConfig::new(GameMode::Soccar)
    .with_rng_seed(42)
    .with_mem_weight_mode(ArenaMemWeightMode::Balanced);
let arena = Arena::new_with_config(config);
```

Load the collision meshes before this code. Use the same initial state,
random seed, and inputs when you need repeatable runs.

### Choose a memory mode

`Heavy` is the default. Try `Balanced` when you need many arenas. Use `Light`
when memory use is more important than simulation speed.

The following approximate measurements come from `stress_v3`. They are
examples, not memory or speed guarantees. Results depend on the platform and
workload.

| Mode | 1v1 memory per arena | 1v1 speed | 3v3 memory per arena | 3v3 speed |
| --- | ---: | ---: | ---: | ---: |
| `Heavy` | 915 KiB | Baseline | 946 KiB | Baseline |
| `Balanced` | 108 KiB | 1–2% slower | 123 KiB | 3% slower |
| `Light` | 66 KiB | 9% slower | 91 KiB | 12% slower |

Use the [benchmark guide](rocketsim_test/README.md#measure-simulation-speed)
to compare memory modes on your workload.

## More examples and checks

Run these examples from the workspace root after you prepare the meshes:

| Task | Command |
| --- | --- |
| Run a 1v1 simulation and read events | `cargo run -p rocketsim --example basic_match` |
| Change mutators, pads, and memory mode | `cargo run -p rocketsim --example custom_mutators` |
| Try the supported game modes | `cargo run -p rocketsim --example game_modes` |
| Read state and cast rays | `cargo run -p rocketsim --example snapshot_raycast` |

- [API reference](https://docs.rs/rocketsim). For this checkout, run `cargo doc -p rocketsim --no-deps --open`.
- [Migrate from 0.2 to 0.3](MIGRATING.md).
- [Tests, recording comparisons, and benchmarks](rocketsim_test/README.md).
- [Viewer screenshot and input review tools](tools/README.md).
- [GitHub Actions and release instructions](tools/ci/README.md).
- [Issue tracker](https://github.com/ZealanL/RocketSim/issues).

## Troubleshooting

- **The mesh directory is missing:** Check the process's working directory.
  Use `rocketsim::init` with an explicit path if necessary.
- **Initialization rejects a mesh set:** Check for missing, duplicate, or
  incompatible `.cmf` components. Do not create the arena until initialization
  returns `Ok(())`.
- **Arena creation fails after initialization:** Confirm that you loaded the
  mesh set for that mode. An omitted mode stays unloaded.
- **No window appears:** This is expected for the core library. Use the viewer
  example or add `rocketsim_vis` to your application.
- **An older example does not compile:** Check its crate version. This README
  uses the 0.3 API. Do not mix 0.2 code with 0.3 dependencies.

The Rust source uses the [MIT license](LICENSE). Viewer assets have separate
terms and source records. Read the [viewer license record](rocketsim_viewer/LICENSES.md)
before redistribution. The record identifies unconfirmed texture terms and a
shader attribution question. The [model terms](rocketsim_viewer/src/models/MODEL_LICENSE_README.md)
remain unchanged.
