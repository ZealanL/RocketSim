use crate::{Team, consts::dropshot};

/// Dropshot tile health: `Full -> Damaged -> Broken` (broken = no collision).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum TileDamageState {
    #[default]
    Full,
    Damaged,
    Broken,
}

/// All Dropshot tiles: `states[0]` = Blue (70), `states[1]` = Orange (70).
///
/// Read with `Arena::get_tile_states`, write with `Arena::set_tile_states`.
/// `DEFAULT` is all-`Full`. Index tiles via [`TileStates::get_team_states`].
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
    /// All tiles undamaged.
    pub const DEFAULT: Self = Self {
        states: [[TileDamageState::Full; dropshot::NUM_TILES_PER_TEAM]; 2],
    };

    /// The 70 tiles for one team.
    pub fn get_team_states(&self, team: Team) -> &[TileDamageState; dropshot::NUM_TILES_PER_TEAM] {
        match team {
            Team::Blue => &self.states[0],
            Team::Orange => &self.states[1],
        }
    }
}
