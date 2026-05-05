use crate::{BallState, BoostPadConfig, BoostPadState, CarInfo, CarState, GameMode};

#[derive(Debug, Clone)]
pub struct ArenaState {
    pub(crate) game_mode: GameMode,
    pub tick_count: u64,
    pub cars: Vec<(CarInfo, CarState)>,
    pub ball: BallState,
    pub boost_pads: Vec<(BoostPadConfig, BoostPadState)>,
}

impl ArenaState {
    #[must_use]
    pub const fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    #[must_use]
    pub fn new_empty(game_mode: GameMode) -> Self {
        Self {
            game_mode,
            tick_count: 0,
            cars: Vec::new(),
            ball: BallState::default(),
            boost_pads: Vec::new(),
        }
    }

    #[must_use]
    pub const fn num_cars(&self) -> usize {
        self.cars.len()
    }

    #[must_use]
    pub const fn num_boost_pads(&self) -> usize {
        self.boost_pads.len()
    }
}
