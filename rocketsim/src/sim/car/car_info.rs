use crate::{CarBodyConfig, Team};

/// Car information: current arena index, team, and hitbox preset.
///
/// Returned by `Arena::get_car_info`. [`crate::Car`] derefs to this type.
/// The index can change after [`crate::Arena::remove_car`].
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CarInfo {
    pub idx: usize,
    pub team: Team,
    pub config: CarBodyConfig,
}
