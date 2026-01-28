use crate::{CarBodyConfig, Team};

/// Immutable information attached to each car
#[derive(Clone, Copy, Debug, Default)]
pub struct CarInfo {
    pub idx: usize,
    pub team: Team,
    pub config: CarBodyConfig,
}