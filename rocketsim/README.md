# rocketsim

Use `rocketsim` to simulate Rocket League car and ball physics without
starting the game. It supports bot testing, agent training, and ball-motion
prediction. Your application controls the simulation and reads its state.
The library does not open a window.


Enable the optional `serde` feature if you need to serialize simulation types.
The core library does not need a C++ compiler or graphics libraries.

## Prepare the collision meshes

Supply compatible `.cmf` collision meshes before you create an arena. The
crate does not include these files. Use the following directory layout:

```text
collision_meshes/
  soccar/     16 .cmf files
  hoops/      12 .cmf files
  dropshot/    2 .cmf files
```

You only need the mesh sets for the modes you use. Soccar, Snowday, and
Heatseeker share the Soccar set. `TheVoid` does not need meshes.

`init_from_default` reads `./collision_meshes/` relative to the process's
working directory. Use `rocketsim::init` for another directory. Use
`init_from_mem` to supply `.cmf` byte buffers.

Load all required modes on the first successful initialization. Later calls
do not replace the loaded meshes. You can retry a failed initialization.
Handle its error before you create an arena. Creating an arena for an
unloaded mode panics.

Mesh files are external assets. Review their source and license before you
redistribute them.

## Run a simulation

This complete program creates a car, applies throttle, and runs one simulated
second. Run it from the directory that contains `collision_meshes/`:

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

Each `step_tick` advances the simulation by `1/120` second. It does not wait
for real time. Omit sleeps when you need the simulation to run as fast as
possible.

### Apply controls and read results

- Set inputs with `set_car_controls` before a tick. Controls remain active
  until you replace them. Send `CarControls::default()` to release all inputs.
- Release `jump` before you press it again for another jump or flip.
- Read car and ball state with `get_car_state` and `get_ball_state`.
- Read collision and boost-pickup events from the slice returned by
  `step_tick`. Copy events you need to keep before you step again.
- Check `is_ball_scored` after stepping. Your application must keep the match
  score and reset play after a goal.

## Configure an arena

Use `ArenaConfig` to select a game mode, mutators, memory mode, and random
seed. After you load the meshes, you can create an arena like this:

```rust,no_run
use rocketsim::{Arena, ArenaConfig, ArenaMemWeightMode, GameMode};

let config = ArenaConfig::new(GameMode::Soccar)
    .with_rng_seed(42)
    .with_mem_weight_mode(ArenaMemWeightMode::Balanced);
let mut arena = Arena::new_with_config(config);
```

Supported modes are Soccar, Hoops, Dropshot, Snowday, Heatseeker, and `TheVoid`.
Use the same initial state, seed, and inputs for repeatable runs.

Choose a memory mode for your workload. `Heavy` is the default.

The following approximate measurements come from `stress_v3`. They are
examples, not memory or speed guarantees. Results depend on the platform and
workload.

| Mode | 1v1 memory per arena | 1v1 speed | 3v3 memory per arena | 3v3 speed |
| --- | ---: | ---: | ---: | ---: |
| `Heavy` | 915 KiB | Baseline | 946 KiB | Baseline |
| `Balanced` | 108 KiB | 1–2% slower | 123 KiB | 3% slower |
| `Light` | 66 KiB | 9% slower | 91 KiB | 12% slower |

Use `MutatorConfig` to change rules and physics settings. `ArenaConfig` also
accepts a custom boost-pad layout. See the
[API reference](https://docs.rs/rocketsim) for configuration, state, event,
and raycast APIs.

## Units and coordinates

- Positions and dimensions use Unreal units. There are 50 Unreal units per
  meter. Linear velocities use Unreal units per second.
- `+Z` points up. Blue defends the `-Y` goal; Orange defends the `+Y` goal.
- Angular velocities use radians per second.

## View your simulation

Viewing is optional. Add [`rocketsim_vis`](https://crates.io/crates/rocketsim_vis)
to send frames to the separate
[`rocketsim_viewer`](https://crates.io/crates/rocketsim_viewer) application.
Your simulation does not need the viewer's graphics dependencies.
