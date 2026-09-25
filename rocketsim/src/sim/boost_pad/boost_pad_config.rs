use glam::Vec3A;

/// Static boost-pad layout entry: world position (uu) + size class.
///
/// Pad order after [`crate::Arena`] construction is RLBot/RLGym (Y, then X),
/// not the input order — always resolve indices via
/// `Arena::get_boost_pad_config`.
#[derive(Clone, Copy, Debug, Default)]
pub struct BoostPadConfig {
    pub pos: Vec3A,
    pub is_big: bool,
}
