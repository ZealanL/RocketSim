use crate::sim::BoostPad;
use crate::{Arena, BoostPadConfig, BoostPadState};

impl Arena {
    pub fn get_boost_pad_state(&self, idx: usize) -> BoostPadState {
        let pad = self.boost_pads()[idx];

        // TODO: Store a lot of these variables within the config maybe? Unsure
        let cooldown = if let Some(gave_boost_tick) = pad.gave_boost_tick_count {
            let max_cooldown = pad.max_cooldown;
            let time_since = (self.tick_count() - gave_boost_tick) as f32 * self.tick_time();
            (max_cooldown - time_since).max(0.0)
        } else {
            0.0
        };

        BoostPadState { cooldown }
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        let tick_rate = self.tick_rate();
        let tick_count = self.tick_count;
        let pad = &mut self.boost_pad_grid.all_pads[idx];
        if state.cooldown > 0.0 {
            let time_since_pickup = (pad.max_cooldown - state.cooldown).max(0.0);
            let ticks_since_pickup = (time_since_pickup * tick_rate).round() as u64;
            pad.gave_boost_tick_count = Some(tick_count - ticks_since_pickup);
        } else {
            self.boost_pad_grid.all_pads[idx].gave_boost_tick_count = None;
        }
    }

    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }

    pub(crate) fn boost_pads(&self) -> &[BoostPad] {
        &self.boost_pad_grid.all_pads
    }

    pub fn num_boost_pads(&self) -> usize {
        self.boost_pads().len()
    }

    pub fn get_all_boost_pad_states(&self) -> Vec<BoostPadState> {
        (0..self.num_boost_pads())
            .into_iter()
            .map(|i| self.get_boost_pad_state(i))
            .collect()
    }
}
