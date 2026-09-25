/// Blue (`0`) or Orange (`1`).
///
/// Blue defends `-Y` and kicks off facing `+Y`; Orange is mirrored.
/// Convert from a world `y` with [`Team::from_team_y`], or from a car index
/// with `Team::try_from(car_idx % 2)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Team {
    #[default]
    Blue,
    Orange,
}

impl Team {
    /// All teams, in index order (`[Blue, Orange]`).
    pub const ALL: [Self; 2] = [Self::Blue, Self::Orange];

    /// Returns `true` for [`Team::Blue`].
    #[must_use]
    pub const fn is_blue(self) -> bool {
        (self as usize) == (Self::Blue as usize)
    }

    /// Returns `true` for [`Team::Orange`].
    #[must_use]
    pub const fn is_orange(self) -> bool {
        (self as usize) == (Self::Orange as usize)
    }

    /// Picks a team from a world-space `y`: `y <= 0` is Blue, else Orange.
    #[must_use]
    pub const fn from_team_y(y: f32) -> Self {
        if y <= 0.0 { Self::Blue } else { Self::Orange }
    }

    /// `-1.0` for Blue, `+1.0` for Orange.
    ///
    /// This is the sign used to mirror spawn positions/velocities across the
    /// field center (see [`crate::Arena::reset_to_random_kickoff`]).
    #[must_use]
    pub const fn get_y_dir(self) -> f32 {
        (self as i8 * 2 - 1) as f32
    }

    /// Returns the other team.
    #[must_use]
    pub const fn opposite(self) -> Self {
        match self {
            Self::Blue => Self::Orange,
            Self::Orange => Self::Blue,
        }
    }
}

impl TryFrom<u8> for Team {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Blue),
            1 => Ok(Self::Orange),
            _ => Err(()),
        }
    }
}
