use glam::Vec3A;

use crate::{BoostPadConfig, GameMode, MutatorConfig};

#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Balanced,
    Light,
}

/// How wheel suspension rays are resolved against the arena.
///
/// The reference RocketSim has two suspension-collision paths: the default
/// `btVehicleRaycaster::castRay` against the whole collision world (static
/// planes, arena trimesh, and dynamic bodies), and the
/// `SuspensionCollisionGrid` fast path whose unmarked cells resolve against
/// the analytic arena planes only (floor, ceiling, side walls). Captures
/// produced by planes-only suspension engines exist in the wild; their wheel
/// stream is only reproducible under the analytic model.
#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum WheelRaycastMode {
    /// Full collision-world raycast (C++ `btVehicleRL::rayCast` default).
    #[default]
    World,
    /// Analytic arena planes only — the `SuspensionCollisionGrid`
    /// unmarked-cell path applied to every wheel ray. The trimesh and
    /// dynamic bodies are ignored by the suspension rays.
    ArenaPlanes,
}

#[derive(Clone, Debug)]
pub struct ArenaConfig {
    pub game_mode: GameMode,
    pub mutators: MutatorConfig,
    pub mem_weight_mode: ArenaMemWeightMode,
    /// How wheel suspension rays resolve contacts; see [`WheelRaycastMode`].
    pub wheel_raycast_mode: WheelRaycastMode,
    pub min_pos: Vec3A,
    pub max_pos: Vec3A,
    pub max_aabb_len: f32,
    pub no_ball_rot: bool,
    /// Use a custom list of boost pads (`custom_boost_pads`) instead of the normal one
    pub custom_boost_pads: Option<Vec<BoostPadConfig>>,
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
        game_mode: GameMode::Soccar,
        mutators: MutatorConfig::new(GameMode::Soccar),
        mem_weight_mode: ArenaMemWeightMode::Heavy,
        wheel_raycast_mode: WheelRaycastMode::World,
        min_pos: Vec3A::new(-5600., -6000., 0.),
        max_pos: Vec3A::new(5600., 6000., 2200.),
        max_aabb_len: 370.,
        no_ball_rot: false,
        custom_boost_pads: None,
        rng_seed: None,
    };

    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self {
            game_mode,
            mutators: MutatorConfig::new(game_mode),
            ..Self::DEFAULT
        }
    }

    #[must_use]
    pub fn with_mutators(mut self, mutators: MutatorConfig) -> Self {
        self.mutators = mutators;
        self
    }

    #[must_use]
    pub fn with_mem_weight_mode(mut self, mem_weight_mode: ArenaMemWeightMode) -> Self {
        self.mem_weight_mode = mem_weight_mode;
        self
    }

    #[must_use]
    pub fn with_wheel_raycast_mode(mut self, wheel_raycast_mode: WheelRaycastMode) -> Self {
        self.wheel_raycast_mode = wheel_raycast_mode;
        self
    }

    #[must_use]
    pub fn with_min_pos(mut self, min_pos: Vec3A) -> Self {
        self.min_pos = min_pos;
        self
    }

    #[must_use]
    pub fn with_max_pos(mut self, max_pos: Vec3A) -> Self {
        self.max_pos = max_pos;
        self
    }

    #[must_use]
    pub fn with_max_aabb_len(mut self, max_aabb_len: f32) -> Self {
        self.max_aabb_len = max_aabb_len;
        self
    }

    #[must_use]
    pub fn with_no_ball_rot(mut self, no_ball_rot: bool) -> Self {
        self.no_ball_rot = no_ball_rot;
        self
    }

    #[must_use]
    pub fn with_custom_boost_pads(mut self, custom_boost_pads: Vec<BoostPadConfig>) -> Self {
        self.custom_boost_pads = Some(custom_boost_pads);
        self
    }

    #[must_use]
    pub fn with_rng_seed(mut self, rng_seed: u64) -> Self {
        self.rng_seed = Some(rng_seed);
        self
    }
}
