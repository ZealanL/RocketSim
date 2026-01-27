use glam::Vec3;

#[derive(Clone, Copy, Debug, PartialEq)]

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

    pub const fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.steer = self.steer.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }

    pub const fn pyr(self) -> Vec3 {
        Vec3::new(self.pitch, self.yaw, self.roll)
    }

    //////////////////////////

    pub const fn with_throttle(mut self, val: f32) -> Self {
        self.throttle = val;
        self
    }

    pub const fn with_steer(mut self, val: f32) -> Self {
        self.steer = val;
        self
    }

    pub const fn with_pitch(mut self, val: f32) -> Self {
        self.pitch = val;
        self
    }

    pub const fn with_yaw(mut self, val: f32) -> Self {
        self.yaw = val;
        self
    }

    pub const fn with_roll(mut self, val: f32) -> Self {
        self.roll = val;
        self
    }

    pub const fn with_pyr(mut self, pyr: Vec3) -> Self {
        (self.pitch, self.yaw, self.roll) = (pyr.x, pyr.y, pyr.z);
        self
    }

    pub const fn with_jump(mut self, val: bool) -> Self {
        self.jump = val;
        self
    }

    pub const fn with_boost(mut self, val: bool) -> Self {
        self.boost = val;
        self
    }

    pub const fn with_handbrake(mut self, val: bool) -> Self {
        self.handbrake = val;
        self
    }
}
