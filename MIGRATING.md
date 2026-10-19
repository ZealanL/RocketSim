# Migrate RocketSim from 0.2 to 0.3

## Update car-state access and replacement

`CarState` contact and previous-control fields are now private cache data.
Replace direct field reads with getters:

```rust
let grounded = state.is_on_ground();
let wheel_hits = state.wheels_with_contact();
let previous_controls = state.prev_controls();
let world_normal = state.world_contact_normal();
```


Use `set_contact_flags` to import ground and wheel flags. True wheel flags get
placeholder hit data, not measured raycasts. Use `set_prev_controls` inside
`edit_car_state_full` when replay needs prior input history. Normal
`set_car_state` sets previous controls to the current controls. The world
contact normal has a getter but no public setter. Restore a full snapshot to
preserve it.

`Arena::set_car_state` now takes a `CarCore`, not a full `CarState`. It derives
tuning from velocity and clears transient contacts. Use it for normal state
changes:

```rust
use rocketsim::CarCore;

arena.set_car_state(car_idx, CarCore::grounded(phys, 50.0));
```

Use `Arena::set_car_state_full` to restore a saved `CarState`, including its
tuning and contact/input cache. The setter clears cached body impulses. If
`snapshot` is a saved `CarState`, replace the old `set_car_state` call:

```rust
arena.set_car_state_full(car_idx, snapshot);
```

Use `Arena::edit_car_state_full` to change a full state in place.

The 0.3 API adds `CarCore` for primary car state and `CarTuning` for replay
tuning data. A `CarState` cannot be created with an external struct literal
because its cache is private. Use `CarCore` builders for normal placement, or
edit a `CarState::DEFAULT` value or a copied snapshot.

## Update direct `glam` dependencies

Public math types now use `glam` 0.34 instead of 0.33.5. Update a direct
`glam` dependency if your code passes `glam` values into RocketSim.

## Retry failed mesh initialization

Initialization now validates all supplied mesh sets before making them
available. It rejects missing, duplicate, or unknown components. A failed load
leaves no partial state. Correct the input and retry initialization. In
0.2.12, a failed load could leave initialization blocked.

## Connect to the separate viewer

`rocketsim_vis` no longer opens a renderer in the simulation process. It
connects to the separate `rocketsim-viewer` application. Install the matching
viewer release:

```sh
cargo install --locked rocketsim_viewer --version 0.3.0
```

Make `rocketsim-viewer` available on `PATH`, or set `ROCKETSIM_VIEWER` to its
executable path.

Import `ArenaVisExt` and use the fallible attach method when your application
must handle startup errors:

```rust
use rocketsim_vis::ArenaVisExt;

arena.try_set_vis_enabled(true)?;
```

This starts or reuses the viewer at the default address. Use
`try_set_net_vis(Some(address))?` to connect to a viewer you started yourself.
The non-fallible `set_vis_enabled` method reports setup errors to standard
error.

`VisInst`, `Vis`, and `backend` are removed. Custom code that used those
in-process renderer types must use the separate viewer and its transport API.

## Apply viewer controls in your simulation

The viewer sends controls, but it does not apply them to an arena. Read the
latest controls and set them before each tick. This example controls one car
and uses neutral input when no fresh message exists:

```rust
use rocketsim::CarControls;
use rocketsim_vis::ArenaVisExt;

let controls = arena
    .latest_vis_controls()
    .map_or_else(CarControls::default, |message| message.controls);
arena.set_car_controls(car_idx, controls);
arena.step_tick();
```

For multiple cars, check `message.car_index` before you apply its controls.
Handle requests from `take_vis_actions` in your own code. A request does not
run a reset, dribble, or ball launch by itself.


## Use retained debug drawings

The new `DebugRenderer` sends drawings to the separate viewer. After you
attach a viewer, use `draw_frame` for temporary marks and a named group for
persistent drawings:

```rust
use rocketsim_vis::{
    ArenaVisExt,
    debug::{Color, DebugRenderer},
};

let renderer: DebugRenderer = arena.debug_renderer().expect("viewer not attached");
renderer
    .draw_frame(|draw| {
        draw.line([0.0, 0.0, 0.0], [100.0, 0.0, 0.0])
            .color(Color::RED);
    })
    .expect("valid drawing");
renderer
    .group("target")
    .draw(|draw| {
        draw.point([0.0, 0.0, 0.0]);
    })
    .expect("valid drawing");
```

A frame drawing remains through the next simulation update and expires at the
following update. A named group persists until you replace or remove it.
