# rocketsim_vis_net

Lightweight network interface for the RocketSim detached viewer: frame
streaming, driving inputs, bindings, and the viewer launcher. No window,
no renderer — sim crates depend on this, not on `rocketsim_vis`.

Enable it on any `rocketsim::Arena` and it streams ticks to a viewer:

```rust,no_run
use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
use rocketsim_vis_net::ArenaVisExt;

init_from_default(true).unwrap();
let mut arena = Arena::new(GameMode::Soccar);
arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
arena.reset_to_random_kickoff(None);
arena.set_vis_enabled(true);

for _ in 0..240 {
    arena.step_tick();
    std::thread::sleep(std::time::Duration::from_millis(8));
}
arena.set_vis_enabled(false);
```

The viewer itself lives in `rocketsim_vis` (`vis_viewer` binary).
Wire format is in `src/proto.rs`.
