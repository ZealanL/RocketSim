use glam::Vec3A;

/// Static boost-pad layout entry: world position (uu) + size class.
///
/// Custom layouts must use non-overlapping XY acquisition circles. Their
/// radii are 208 uu for big pads and 144 uu for small pads. A car can touch
/// only one pad at a time. Do not place pads so one configured car hitbox can
/// overlap multiple pad volumes. Pad order after [`crate::Arena`] construction
/// is RLBot/RLGym (Y, then X), not the input order. Resolve indices with
/// `Arena::get_boost_pad_config`.
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BoostPadConfig {
    pub pos: Vec3A,
    pub is_big: bool,
}
