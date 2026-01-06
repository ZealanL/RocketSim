#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Team {
    #[default]
    Blue,
    Orange,
}

impl Team {
    pub const ALL: [Team; 2] = [Team::Blue, Team::Orange];

    pub const fn from_team_y(y: f32) -> Self {
        if y <= 0.0 { Self::Blue } else { Self::Orange }
    }

    pub fn get_y_dir(self) -> f32 {
        f32::from(self as i8 * 2 - 1)
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
