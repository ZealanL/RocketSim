/// Dynamic boost-pad state: seconds of cooldown left (`0` = active).
///
/// Read with `Arena::get_boost_pad_state`, force with
/// `Arena::set_boost_pad_state`. Cooldowns tick in
/// [`crate::Arena::step_tick`] from `MutatorConfig::boost_pad_cooldown_*`.
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
    /// Active-pad placeholder (`cooldown == 0`).
    pub const DEFAULT: Self = Self { cooldown: 0.0 };

    /// Returns `true` when the pad can be picked up (`cooldown <= 0`).
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.cooldown <= 0.0
    }
}
