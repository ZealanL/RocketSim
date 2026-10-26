/// Dynamic boost-pad state: seconds of cooldown left (`0` = active) and ordered car members.
///
/// This public type is no longer `Copy`. Existing struct literals must now provide
/// `touching_car_idxs` or use `BoostPadState::DEFAULT`. The added field changes
/// serialized snapshots. Map-based formats can default a missing field, but the
/// next tick rebuilds order by car index and loses saved arrival order. Sequence
/// formats such as postcard cannot read old layouts. Rebuild both protocol ends.
/// Read with `Arena::get_boost_pad_state`; restore with
/// `Arena::set_boost_pad_state`. A car index must occur in no more than one
/// pad state's `touching_car_idxs` list. If restored data repeats an index
/// across pads, the next tick keeps it on the lowest-index pad that still
/// retains the car. Cooldowns tick in
/// [`crate::Arena::step_tick`] from `MutatorConfig::boost_pad_cooldown_*`.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoostPadState {
    /// Car members in retained-entry order. A car index can occur on one pad only.
    #[cfg_attr(feature = "serde", serde(default))]
    pub touching_car_idxs: Vec<usize>,
    /// Seconds of cooldown left (`0` = active).
    pub cooldown: f32,
}

impl Default for BoostPadState {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl BoostPadState {
    /// Active-pad placeholder (`cooldown == 0`).
    pub const DEFAULT: Self = Self {
        cooldown: 0.0,
        touching_car_idxs: Vec::new(),
    };

    /// Returns `true` when the pad can be picked up (`cooldown <= 0`).
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.cooldown <= 0.0
    }
}
