use glam::{IVec3, Vec3A};

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

/// Asymmetric 8-bit input quantization: negatives scale by 128, positives by 127.
pub(super) fn quantize_axis_inputs(ctrls: Vec3A) -> Vec3A {
    const UPPER_BOUND: Vec3A = Vec3A::splat(128.0);
    const LOWER_BOUND: Vec3A = Vec3A::splat(127.0);

    let clamped = ctrls.clamp(Vec3A::NEG_ONE, Vec3A::ONE);
    let scale = Vec3A::select(clamped.cmplt(Vec3A::ZERO), UPPER_BOUND, LOWER_BOUND);
    let biased = clamped * scale + UPPER_BOUND;
    let w = biased + biased + Vec3A::splat(0.5);
    let byte = ((w.round().as_ivec3() >> 1i32) & IVec3::splat(0xFF)).as_vec3a();
    let s = byte - UPPER_BOUND;

    Vec3A::select(
        s.cmplt(Vec3A::ZERO),
        s * (1.0 / UPPER_BOUND),
        s / LOWER_BOUND,
    )
}

/// Single-axis form of [`quantize_axis_inputs`].
#[must_use]
pub(super) fn quantize_axis_input(x: f32) -> f32 {
    let clamped = x.clamp(-1.0, 1.0);
    let y = if clamped < 0.0 {
        (clamped * 128.0).max(-128.0)
    } else {
        (clamped * 127.0).min(127.0)
    };
    let w = ((y + 128.0) + (y + 128.0)) + 0.5;
    let eax = w.round_ties_even() as i32;
    let byte = ((eax >> 1) & 0xFF) as u8;
    let s = (byte as f32) - 128.0;
    if byte < 0x80 {
        s * (1.0 / 128.0)
    } else {
        s / 127.0
    }
}
