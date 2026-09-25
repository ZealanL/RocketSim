use crate::{CarBodyConfig, Team};

/// Immutable identity for a car: arena index, team, and hitbox preset.
///
/// Returned by `Arena::get_car_info`; the index matches `add_car` order and
/// [`crate::Car`] derefs to this.
#[derive(Clone, Copy, Debug, Default)]
pub struct CarInfo {
    pub idx: usize,
    pub team: Team,
    pub config: CarBodyConfig,
}
