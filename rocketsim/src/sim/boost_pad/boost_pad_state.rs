#[derive(Clone, Copy, Debug)]
pub struct BoostPadState {
    /// The last tick when we gave a car boost
    pub cooldown: f32,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    pub const DEFAULT: Self = Self { cooldown: 0.0 };

    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.cooldown <= 0.0
    }
}
