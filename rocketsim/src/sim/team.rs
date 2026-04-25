#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Team {
    #[default]
    Blue,
    Orange,
}

impl Team {
    pub const ALL: [Self; 2] = [Self::Blue, Self::Orange];

    #[must_use]
    pub const fn is_blue(self) -> bool {
        (self as usize) == (Self::Blue as usize)
    }

    #[must_use]
    pub const fn is_orange(self) -> bool {
        (self as usize) == (Self::Orange as usize)
    }

    #[must_use]
    pub const fn from_team_y(y: f32) -> Self {
        if y <= 0.0 { Self::Blue } else { Self::Orange }
    }

    #[must_use]
    pub const fn get_y_dir(self) -> f32 {
        (self as i8 * 2 - 1) as f32
    }

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
