use crate::{Team, consts::dropshot};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum TileDamageState {
    #[default]
    Full,
    Damaged,
    Broken,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TileStates {
    pub states: [[TileDamageState; dropshot::NUM_TILES_PER_TEAM]; 2],
}

impl Default for TileStates {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl TileStates {
    pub const DEFAULT: Self = Self {
        states: [[TileDamageState::Full; dropshot::NUM_TILES_PER_TEAM]; 2],
    };

    pub fn get_team_states(&self, team: Team) -> &[TileDamageState; dropshot::NUM_TILES_PER_TEAM] {
        match team {
            Team::Blue => &self.states[0],
            Team::Orange => &self.states[1],
        }
    }
}
