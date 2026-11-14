# rocketsim

Use `rocketsim` to simulate Rocket League car and ball physics without
starting the game. It supports bot testing, agent training, and ball-motion
prediction. Your application controls the simulation and reads its state.
The library does not open a window.


Enable the optional `serde` feature if you need to serialize simulation types.
The core library does not need a C++ compiler or graphics libraries.

For an existing application, see the
[0.2-to-0.3 migration guide](https://github.com/ZealanL/RocketSim/blob/v0.3.0/MIGRATING.md).

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

`Balanced` is the default memory mode. Use `Light` to reduce memory use.
`Heavy` uses smaller ray-query cells and reserves more handles per cell.
It does not guarantee higher speed. Compare modes on your workload.

The following values are approximate Linux RSS measurements on a Ryzen 9
7940HS. After mesh initialization and worker startup, create 100 Soccar arenas
with Octane cars and default mutators. Run 100 episodes of 120 ticks per
arena with fixed generated controls on 16 workers, one per logical CPU.
Use platform math (the default), SAT, and the `parallel` feature. Subtract
initial RSS from live RSS and divide by 100 while the arenas are still alive.
The 1v1 workload uses two cars per arena. The 3v3 workload uses six.
Use the median of three separate processes for each entry.

| Mode | 1v1 memory per arena | 3v3 memory per arena |
| --- | ---: | ---: |
| `Heavy` | 1,062 KiB | 1,076 KiB |
| `Balanced` | 86 KiB | 102 KiB |
| `Light` | 51 KiB | 65 KiB |

These values are not upper bounds. RSS includes allocator retention and
shared pages. This run measures 12,000 ticks per arena. Do not compare these
values directly with the earlier 1,000,000-tick measurement. The input sequence
also differs. This memory probe does not measure simulation speed.

The separate speed screen uses 512 arenas on 16 workers. It replays 32
120-tick windows from each of the Nexto 1v1 and Wisp 3v3 recordings. Builds
use `native-math`, SAT, and the `parallel` feature. Each entry is the median
of three samples. Controls and the full per-tick batch join are timed.
Construction, restore, and result reads are not timed.

| Mode | 1v1 thousand arena-ticks/s | 3v3 thousand arena-ticks/s |
| --- | ---: | ---: |
| `Heavy` | 2,237 | 977 |
| `Balanced` | 2,312 | 1,060 |
| `Light` | 2,478 | 1,191 |

This short screen does not establish a speed order for all workloads.
Measure your own roster, arena count, and worker count.

`Light` shares a packed static-AABB table for ray culling. Dynamic proxies,
retained contacts, events, pad timers, and random state remain local to each
arena. `Balanced` and `Heavy` keep their existing query path.

### Step many arenas in parallel

Enable the optional `parallel` feature to use `Arena::step_batch` and
`Arena::step_batch_with`:

```toml
[dependencies]
rocketsim = { version = "0.3", features = ["parallel"] }
```

An empty batch or a batch with one arena runs on the calling thread. It does
not use the Rayon pool. A batch with two or more arenas uses the current Rayon
pool. By default, the current pool is the process-global pool. Set
`RAYON_NUM_THREADS` before the first parallel call to choose its size. Use
`pool.install(|| Arena::step_batch(&mut arenas))` to select a local pool.

Each arena keeps its own physics state and random generator. The worker that
runs an arena does not change its result. The arena order across workers is not
specified. A panic does not roll back the batch. Other arenas can continue, and
each arena that finishes keeps its new state. The batch can make partial
progress.

The batch methods do not use `ArenaStepBuffers` automatically. Use your own
worker loop with `step_tick` or `step_tick_with_buffers` when you need to
control buffer reuse. Measure the parallel option on your workload.

### Run many arenas on each worker

Use `ArenaStepBuffers` to reuse temporary solver and contact buffers between
arenas. Create one buffer set for each worker. Reuse it for every tick:

```rust,no_run
use rocketsim::{Arena, ArenaStepBuffers};

fn step_worker(arenas: &mut [Arena], buffers: &mut ArenaStepBuffers) {
    for arena in arenas {
        arena.step_tick_with_buffers(buffers);
    }
}
```

The arena still owns its physics state, retained contacts, events, and random
generator. Buffer reuse does not change the physics result. It does not create
worker threads. Drop the buffer set to release its allocations. Arenas that
also use `step_tick` retain their own grown buffers.

Use ordinary `step_tick` for one arena per worker. The buffer exchange adds
work and does not help every arena count. Compare both methods with your
roster, memory mode, arena count, and worker count. The memory-mode table above
uses `step_batch`, which calls ordinary `step_tick`; it does not measure
buffer reuse.

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

## License

The source uses the [MIT license](LICENSE). Collision meshes are external
inputs with separate terms. This crate does not include them.
