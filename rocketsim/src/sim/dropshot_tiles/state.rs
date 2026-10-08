use crate::{Team, consts::dropshot};

/// Dropshot tile health: `Full -> Damaged -> Broken` (broken = no collision).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

#[cfg(feature = "serde")]
impl serde::Serialize for TileStates {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("TileStates", 1)?;
        let nested: Vec<Vec<TileDamageState>> =
            self.states.iter().map(|team| team.to_vec()).collect();
        state.serialize_field("states", &nested)?;
        state.end()
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for TileStates {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        #[derive(serde::Deserialize)]
        struct TileStatesHelper {
            states: Vec<Vec<TileDamageState>>,
        }

        let helper = TileStatesHelper::deserialize(deserializer)?;
        if helper.states.len() != 2 {
            return Err(serde::de::Error::invalid_length(
                helper.states.len(),
                &"2 teams",
            ));
        }
        let mut states = [[TileDamageState::Full; dropshot::NUM_TILES_PER_TEAM]; 2];
        for (team_idx, team) in helper.states.into_iter().enumerate() {
            if team.len() != dropshot::NUM_TILES_PER_TEAM {
                return Err(serde::de::Error::invalid_length(
                    team.len(),
                    &"70 tiles per team",
                ));
            }
            for (tile_idx, tile) in team.into_iter().enumerate() {
                states[team_idx][tile_idx] = tile;
            }
        }
        Ok(Self { states })
    }
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
