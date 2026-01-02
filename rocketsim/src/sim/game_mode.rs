use ahash::AHashMap;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum GameMode {
    #[default]
    Soccar,
    Hoops,
    Heatseeker,
    Snowday,
    Dropshot,
    /// Soccar but without goals, boost pads, or the arena hull. The cars and ball will fall infinitely.
    TheVoid,
}

impl GameMode {
    const NAMES: [&'static str; 7] = [
        "soccar",
        "hoops",
        "heatseeker",
        "snowday",
        "dropshot",
        "dropshot",
        "void",
    ];

    #[must_use]
    pub const fn name(self) -> &'static str {
        Self::NAMES[self as usize]
    }

    pub(crate) fn get_hashes(self) -> AHashMap<u32, u32> {
        macro_rules! zero_iter {
            ($($i:literal),+) => {
                [
                    $(($i, 0)),+
                ].into_iter()
            }
        }

        match self {
            Self::Soccar => zero_iter![
                0xA160_BAF9,
                0x2811_EEE8,
                0xB81A_C8B9,
                0x7603_58D3,
                0x73AE_4940,
                0x918F_4A4E,
                0x1F8E_E550,
                0x255B_A8C1,
                0x14B8_4668,
                0xEC75_9EBF,
                0x94FB_0D5C,
                0xDEA0_7102,
                0xBD4F_BEA8,
                0x39A4_7F63,
                0x3D79_D25D,
                0xD84C_7A68
            ]
            .collect(),
            Self::Hoops => zero_iter![
                0x72F2_359E,
                0x5ED1_4A26,
                0xFD5A_0D07,
                0x92AF_A5B5,
                0x0E41_33C7,
                0x399E_8B5F,
                0xBB9D_4FB5,
                0x8C87_FB93,
                0x1CFD_0E16,
                0xE19E_1DF6,
                0x9CA1_79DC,
                0x16F3_CC19
            ]
            .collect(),
            Self::Dropshot => zero_iter![0x7EB0_B2D3, 0x9110_41D2].collect(),
            _ => AHashMap::new(),
        }
    }
}
