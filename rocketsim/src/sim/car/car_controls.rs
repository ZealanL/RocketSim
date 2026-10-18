use glam::Vec3A;

/// Driver inputs applied on the next [`crate::Arena::step_tick`].
///
/// Analog axes are `-1..1` (clamped by [`CarControls::clamp`]); booleans are
/// edge- or level-triggered per field. `jump` only fires on the rising edge
/// (hold to charge, release or 24 ticks to end); `boost`/`handbrake` are level.
/// `steer` doubles as ground yaw; in air use `yaw/pitch/roll` (-1..1).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CarControls {
    pub throttle: f32,
    pub steer: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub roll: f32,
    pub jump: bool,
    pub boost: bool,
    pub handbrake: bool,
}

impl Default for CarControls {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl CarControls {
    /// All-neutral inputs (no throttle/steer, no buttons).
    pub const DEFAULT: Self = Self {
        throttle: 0.0,
        steer: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        roll: 0.0,
        jump: false,
        boost: false,
        handbrake: false,
    };

    /// Number of scalar slots in [`CarControls::to_floats`] (`8`).
    pub const NUM_VALS: usize = 8;

    /// Clamps analog axes to `-1..1` (booleans untouched).
    ///
    /// [`crate::Arena::step_tick`] clamps inputs from [`crate::Arena::set_car_controls`].
    /// Call it yourself when you blend or network inputs.
    #[must_use]
    pub const fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.steer = self.steer.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }

    /// `(pitch, yaw, roll)` as a vector (air control).
    #[must_use]
    pub const fn pyr(self) -> Vec3A {
        Vec3A::new(self.pitch, self.yaw, self.roll)
    }

    /// Packs to `[throttle, steer, pitch, yaw, roll, jump, boost, handbrake]`
    /// (booleans as `0./1.`) for ML/networking.
    #[must_use]
    pub const fn to_floats(&self) -> [f32; Self::NUM_VALS] {
        [
            self.throttle,
            self.steer,
            self.pitch,
            self.yaw,
            self.roll,
            self.jump as u8 as f32,
            self.boost as u8 as f32,
            self.handbrake as u8 as f32,
        ]
    }

    /// Unpacks [`CarControls::to_floats`]; floats `> boolean_thresh` become `true`.
    #[must_use]
    /// `boolean_thresh`: Values above this threshold set `jump`, `boost`, and `handbrake` to `true`.
    pub const fn from_floats(floats: [f32; Self::NUM_VALS], boolean_thresh: f32) -> Self {
        Self {
            throttle: floats[0],
            steer: floats[1],
            pitch: floats[2],
            yaw: floats[3],
            roll: floats[4],

            jump: floats[5] > boolean_thresh,
            boost: floats[6] > boolean_thresh,
            handbrake: floats[7] > boolean_thresh,
        }
    }

    //////////////////////////

    #[must_use]
    pub const fn with_throttle(mut self, val: f32) -> Self {
        self.throttle = val;
        self
    }

    #[must_use]
    pub const fn with_steer(mut self, val: f32) -> Self {
        self.steer = val;
        self
    }

    #[must_use]
    pub const fn with_pitch(mut self, val: f32) -> Self {
        self.pitch = val;
        self
    }

    #[must_use]
    pub const fn with_yaw(mut self, val: f32) -> Self {
        self.yaw = val;
        self
    }

    #[must_use]
    pub const fn with_roll(mut self, val: f32) -> Self {
        self.roll = val;
        self
    }

    #[must_use]
    pub const fn with_pyr(mut self, pyr: Vec3A) -> Self {
        [self.pitch, self.yaw, self.roll] = pyr.to_array();
        self
    }

    #[must_use]
    pub const fn with_jump(mut self, val: bool) -> Self {
        self.jump = val;
        self
    }

    #[must_use]
    pub const fn with_boost(mut self, val: bool) -> Self {
        self.boost = val;
        self
    }

    #[must_use]
    pub const fn with_handbrake(mut self, val: bool) -> Self {
        self.handbrake = val;
        self
    }
}

#[cfg(test)]
mod tests {
    use std::cmp::Ordering;

    use super::CarControls;

    #[test]
    fn signed_zero_is_equal_and_compares_equal_on_every_axis() {
        let cases = [
            (
                CarControls::DEFAULT.with_throttle(0.0),
                CarControls::DEFAULT.with_throttle(-0.0),
            ),
            (
                CarControls::DEFAULT.with_steer(0.0),
                CarControls::DEFAULT.with_steer(-0.0),
            ),
            (
                CarControls::DEFAULT.with_pitch(0.0),
                CarControls::DEFAULT.with_pitch(-0.0),
            ),
            (
                CarControls::DEFAULT.with_yaw(0.0),
                CarControls::DEFAULT.with_yaw(-0.0),
            ),
            (
                CarControls::DEFAULT.with_roll(0.0),
                CarControls::DEFAULT.with_roll(-0.0),
            ),
        ];

        for (positive, negative) in cases {
            assert_eq!(positive, negative);
            assert_eq!(positive.partial_cmp(&negative), Some(Ordering::Equal));
        }
    }

    #[test]
    fn nan_is_unordered_and_not_equal_even_with_equal_prefix() {
        let controls = CarControls::DEFAULT.with_steer(f32::NAN);
        let other = CarControls::DEFAULT.with_steer(f32::NAN);

        assert_ne!(controls, controls);
        assert_ne!(controls, other);
        assert_eq!(controls.partial_cmp(&other), None);
    }

    #[test]
    fn ordering_is_lexicographic_across_axes_and_boolean_controls() {
        let lower_axis = CarControls::DEFAULT.with_throttle(-0.5);
        let higher_axis = CarControls::DEFAULT.with_throttle(0.5);
        assert!(lower_axis < higher_axis);

        let no_jump = CarControls::DEFAULT;
        let jump = CarControls::DEFAULT.with_jump(true);
        assert!(no_jump < jump);

        let no_boost = CarControls::DEFAULT.with_jump(true);
        let boost = no_boost.with_boost(true);
        assert!(no_boost < boost);

        let no_handbrake = CarControls::DEFAULT.with_boost(true);
        let handbrake = no_handbrake.with_handbrake(true);
        assert!(no_handbrake < handbrake);

        let earlier_difference = CarControls::DEFAULT.with_steer(1.0).with_jump(true);
        let later_difference = CarControls::DEFAULT
            .with_steer(1.0)
            .with_jump(true)
            .with_boost(true);
        assert!(earlier_difference < later_difference);
    }
}
