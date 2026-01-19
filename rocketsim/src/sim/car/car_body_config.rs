use glam::Vec3A;

pub const HITBOX_SIZES: [Vec3A; 7] = [
    Vec3A::new(120.507, 86.6994, 38.6591), // OCTANE
    Vec3A::new(130.427, 85.7799, 33.8),    // DOMINUS
    Vec3A::new(131.32, 87.1704, 31.8944),  // PLANK
    Vec3A::new(133.992, 83.021, 32.8),     // BREAKOUT
    Vec3A::new(129.519, 84.6879, 36.6591), // HYBRID
    Vec3A::new(123.22, 79.2103, 44.1591),  // MERC
    Vec3A::new(120.507 + 0.134, 86.6994 + 0.134, 38.6591 + 0.134), // PSYCLOPS
];

pub const HITBOX_OFFSETS: [Vec3A; 7] = [
    Vec3A::new(13.8757, 0.0, 20.755),
    Vec3A::new(9.0, 0.0, 15.75),
    Vec3A::new(9.00857, 0.0, 12.0942),
    Vec3A::new(12.5, 0.0, 11.75),
    Vec3A::new(13.8757, 0.0, 20.755),
    Vec3A::new(11.3757, 0.0, 21.505),
    Vec3A::new(13.8757, 0.0, 15.0),
];

pub const FRONT_WHEEL_RADS: [f32; 7] = [12.5, 12.0, 12.5, 13.5, 12.5, 15.0, 12.5];
pub const BACK_WHEEL_RADS: [f32; 7] = [15.0, 13.5, 17.0, 15.0, 15.0, 15.0, 15.0];

pub const FRONT_WHEEL_SUS_REST: [f32; 7] = [38.755, 33.95, 31.9242, 29.7, 38.755, 39.505, 33.0];
pub const BACK_WHEEL_SUS_REST: [f32; 7] = [37.055, 33.85, 27.9242, 29.666, 37.055, 39.105, 31.3];

pub const FRONT_WHEELS_OFFSET: [Vec3A; 7] = [
    Vec3A::new(51.25, 25.90, 20.755),
    Vec3A::new(50.30, 31.10, 15.75),
    Vec3A::new(49.97, 27.80, 12.0942),
    Vec3A::new(51.50, 26.67, 11.75),
    Vec3A::new(51.25, 25.90, 20.755),
    Vec3A::new(51.25, 25.90, 21.505),
    Vec3A::new(51.25, 5.000, 15.000),
];

pub const BACK_WHEELS_OFFSET: [Vec3A; 7] = [
    Vec3A::new(-33.75, 29.50, 20.755),
    Vec3A::new(-34.75, 33.00, 15.75),
    Vec3A::new(-35.43, 20.28, 12.0942),
    Vec3A::new(-35.75, 35.00, 11.75),
    Vec3A::new(-34.00, 29.50, 20.755),
    Vec3A::new(-33.75, 29.50, 21.505),
    Vec3A::new(-33.75, 29.50, 15.000),
];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelPairConfig {
    /// Radius of both wheels
    pub wheel_radius: f32,
    /// How far out the suspension rests
    pub suspension_rest_length: f32,
    /// Where the wheel actually connects (suspension start position)
    ///
    /// NOTE: Y should ALWAYS be positive. It will be automatically negated when creating the second wheel.
    pub connection_point_offset: Vec3A,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CarBodyConfig {
    /// Full size of hitbox (NOT the half-size/extent)
    pub hitbox_size: Vec3A,
    /// Offset of the hitbox (from it's origin)
    ///
    /// NOTE: Does not effect car's center of mass, that's always at local (0,0,0)
    pub hitbox_pos_offset: Vec3A,
    pub front_wheels: WheelPairConfig,
    pub back_wheels: WheelPairConfig,
    /// Car has three-wheel behavior (psyclops)
    ///
    /// NOTE: The psyclops actually has 4 wheels, the front two are combined
    /// No car actually has only 3 simulated wheels
    pub three_wheels: bool,
    /// (|yaw| + |pitch| + |roll|) will need to be >= this in order to flip
    pub dodge_deadzone: f32,
}

impl Default for CarBodyConfig {
    fn default() -> Self {
        Self::OCTANE
    }
}

impl CarBodyConfig {
    const fn make_car_config(idx: usize, three_wheels: bool) -> Self {
        Self {
            hitbox_size: HITBOX_SIZES[idx],
            hitbox_pos_offset: HITBOX_OFFSETS[idx],
            front_wheels: WheelPairConfig {
                wheel_radius: FRONT_WHEEL_RADS[idx],
                suspension_rest_length: FRONT_WHEEL_SUS_REST[idx],
                connection_point_offset: FRONT_WHEELS_OFFSET[idx],
            },
            back_wheels: WheelPairConfig {
                wheel_radius: BACK_WHEEL_RADS[idx],
                suspension_rest_length: BACK_WHEEL_SUS_REST[idx],
                connection_point_offset: BACK_WHEELS_OFFSET[idx],
            },
            three_wheels,
            dodge_deadzone: 0.5,
        }
    }

    pub const OCTANE: Self = Self::make_car_config(0, false);
    pub const DOMINUS: Self = Self::make_car_config(1, false);
    pub const PLANK: Self = Self::make_car_config(2, false);
    pub const BREAKOUT: Self = Self::make_car_config(3, false);
    pub const HYBRID: Self = Self::make_car_config(4, false);
    pub const MERC: Self = Self::make_car_config(5, false);
    pub const PSYCLOPS: Self = Self::make_car_config(6, true);
}
