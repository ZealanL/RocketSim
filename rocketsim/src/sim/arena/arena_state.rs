use crate::sim::UserInfoTypes::Ball;
use crate::{BallState, BoostPadConfig, BoostPadState, CarInfo, CarState, GameMode};

#[derive(Debug, Clone)]
pub struct ArenaState {
    pub(crate) game_mode: GameMode,
    pub car_infos: Vec<CarInfo>,
    pub car_states: Vec<CarState>,
    pub ball_state: BallState,
    pub boost_pad_configs: Vec<BoostPadConfig>,
    pub boost_pad_states: Vec<BoostPadState>,
}

impl ArenaState {
    pub fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    pub fn new_empty(game_mode: GameMode) -> ArenaState {
        Self {
            game_mode,
            car_infos: Vec::new(),
            car_states: Vec::new(),
            ball_state: BallState::default(),
            boost_pad_states: Vec::new(),
            boost_pad_configs: Vec::new(),
        }
    }

    pub const fn num_cars(&self) -> usize {
        self.car_infos.len()
    }

    pub const fn num_boost_pads(&self) -> usize {
        self.boost_pad_configs.len()
    }
}