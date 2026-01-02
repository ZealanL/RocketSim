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

    #[must_use]
    pub const fn clamp(mut self) -> Self {
        self.throttle = self.throttle.clamp(-1.0, 1.0);
        self.steer = self.steer.clamp(-1.0, 1.0);
        self.pitch = self.pitch.clamp(-1.0, 1.0);
        self.yaw = self.yaw.clamp(-1.0, 1.0);
        self.roll = self.roll.clamp(-1.0, 1.0);
        self
    }
}
