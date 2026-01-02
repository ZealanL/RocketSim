use crate::BoostPadConfig;
use glam::Vec3A;

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Light,
}

#[derive(Clone, Debug)]
pub struct ArenaConfig {
    pub mem_weight_mode: ArenaMemWeightMode,
    pub min_pos: Vec3A,
    pub max_pos: Vec3A,
    pub max_aabb_len: f32,
    pub no_ball_rot: bool,
    /// Use a custom list of boost pads (`custom_boost_pads`) instead of the normal one
    /// NOTE: This will disable the boost pad grid and will thus worsen performance
    pub use_custom_boost_pads: bool,
    /// Custom boost pads to use, if `use_custom_boost_pads`
    pub custom_boost_pads: Vec<BoostPadConfig>,
    /// Optional RNG seed for deterministic behavior
    /// If None, a random seed will be used
    pub rng_seed: Option<u64>,
}

impl Default for ArenaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ArenaConfig {
    pub const DEFAULT: Self = Self {
        mem_weight_mode: ArenaMemWeightMode::Heavy,
        min_pos: Vec3A::new(-5600., -6000., 0.),
        max_pos: Vec3A::new(5600., 6000., 2200.),
        max_aabb_len: 370.,
        no_ball_rot: true,
        use_custom_boost_pads: false,
        custom_boost_pads: Vec::new(),
        rng_seed: None,
    };
}
