use crate::{BallState, BoostPadConfig, BoostPadState, CarInfo, CarState, GameMode, TileStates};

/// Point-in-time snapshot from [`crate::Arena::get_arena_state`].
///
/// `cars` and `boost_pads` are `(info/config, state)` pairs in arena order.
/// Restore with `Arena::set_ball_state` / `set_car_state` /
/// `set_boost_pad_state` / `set_tile_states`. `tile_states` is `Some` only
/// in Dropshot.
#[derive(Debug, Clone)]
pub struct ArenaState {
    pub(crate) game_mode: GameMode,
    pub tick_count: u64,
    pub cars: Vec<(CarInfo, CarState)>,
    pub ball: BallState,
    pub boost_pads: Vec<(BoostPadConfig, BoostPadState)>,
    pub tile_states: Option<TileStates>,
}

impl ArenaState {
    /// Game mode the snapshot was taken in.
    #[must_use]
    pub const fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    /// Empty placeholder (no cars/pads) for a mode; fill via `get_arena_state`.
    #[must_use]
    pub fn new_empty(game_mode: GameMode) -> Self {
        Self {
            game_mode,
            tick_count: 0,
            cars: Vec::new(),
            ball: BallState::default(),
            boost_pads: Vec::new(),
            tile_states: None,
        }
    }

    /// Number of cars in the snapshot.
    #[must_use]
    pub const fn num_cars(&self) -> usize {
        self.cars.len()
    }

    /// Number of boost pads in the snapshot.
    #[must_use]
    pub const fn num_boost_pads(&self) -> usize {
        self.boost_pads.len()
    }
}
