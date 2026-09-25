use glam::Vec3;

/// Driver inputs applied on the next [`crate::Arena::step_tick`].
///
/// Analog axes are `-1..1` (clamped by [`CarControls::clamp`]); booleans are
/// edge- or level-triggered per field. `jump` only fires on the rising edge
/// (hold to charge, release or 24 ticks to end); `boost`/`handbrake` are level.
/// `steer` doubles as ground yaw; in air use `yaw/pitch/roll` (-1..1).
#[derive(Debug, Clone, Copy)]
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

impl Ord for CarControls {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // TODO: Surely there is a better way to do this
        macro_rules! cmp_field {
            ($field:ident) => {{
                // Explicitly copying because otherwise Rust yells at me
                let a = self.$field;
                let b = other.$field;
                a.total_cmp(&b)
            }};
        }

        cmp_field!(throttle)
            .then_with(|| cmp_field!(steer))
            .then_with(|| cmp_field!(pitch))
            .then_with(|| cmp_field!(yaw))
            .then_with(|| cmp_field!(roll))
            .then_with(|| self.jump.cmp(&other.jump))
            .then_with(|| self.boost.cmp(&other.boost))
            .then_with(|| self.handbrake.cmp(&other.handbrake))
    }
}

impl PartialOrd for CarControls {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for CarControls {
    fn eq(&self, other: &Self) -> bool {
        self.throttle == other.throttle
            && self.steer == other.steer
            && self.pitch == other.pitch
            && self.yaw == other.yaw
            && self.roll == other.roll
            && self.jump == other.jump
            && self.boost == other.boost
            && self.handbrake == other.handbrake
    }
}

impl Eq for CarControls {}

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
    /// [`crate::Arena::set_car_controls`] applies this automatically, but call
    /// it yourself when blending or networking inputs.
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
    pub const fn pyr(self) -> Vec3 {
        Vec3::new(self.pitch, self.yaw, self.roll)
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
    /// `boolean_thresh`: Floats over this value will trigger boolean controls (jump, boost, handbrake)
    pub const fn from_floats(floats: [f32; Self::NUM_VALS], boolean_tresh: f32) -> Self {
        Self {
            throttle: floats[0],
            steer: floats[1],
            pitch: floats[2],
            yaw: floats[3],
            roll: floats[4],

            jump: floats[5] > boolean_tresh,
            boost: floats[6] > boolean_tresh,
            handbrake: floats[7] > boolean_tresh,
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
    pub const fn with_pyr(mut self, pyr: Vec3) -> Self {
        (self.pitch, self.yaw, self.roll) = (pyr.x, pyr.y, pyr.z);
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
