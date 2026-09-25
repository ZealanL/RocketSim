use glam::Vec3A;

use crate::{BoostPadConfig, GameMode, MutatorConfig};

/// Broadphase memory/performance tradeoff for [`crate::Arena`].
///
/// * `Heavy` (default): fastest, ~915 KiB per 1v1 arena in `stress_v3`.
/// * `Balanced`: ~108 KiB, ~1-2% slower.
/// * `Light`: ~66 KiB (single broadphase cell), ~9% slower.
///
/// Numbers vary by platform/workload; see the repo `README.md` table.
#[derive(Clone, Copy, Debug, Default, Hash, PartialEq, Eq)]
pub enum ArenaMemWeightMode {
    #[default]
    Heavy,
    Balanced,
    Light,
}

/// Settings used to build an [`crate::Arena`].
///
/// Start with [`ArenaConfig::new`] (or [`ArenaConfig::DEFAULT` for Soccar)
/// and chain `with_*` builders. All positions are in Unreal units (uu).
#[derive(Clone, Debug)]
pub struct ArenaConfig {
    /// Game mode (arena meshes, goals, ball behavior).
    pub game_mode: GameMode,
    /// Physics/rule tweaks (defaults to `MutatorConfig::new(game_mode)`).
    pub mutators: MutatorConfig,
    /// Broadphase memory/performance tradeoff.
    pub mem_weight_mode: ArenaMemWeightMode,
    /// Broadphase world minimum (uu). Must contain the arena + cars/ball.
    pub min_pos: Vec3A,
    /// Broadphase world maximum (uu).
    pub max_pos: Vec3A,
    /// Target leaf size for the broadphase BVH (uu). Smaller = more memory.
    pub max_aabb_len: f32,
    /// When `true`, the ball never spins (sphere only; useful for debugging).
    pub no_ball_rot: bool,
    /// Use this boost-pad list instead of the mode's default layout.
    /// Pads are re-sorted to RLBot/RLGym order (Y, then X).
    pub custom_boost_pads: Option<Vec<BoostPadConfig>>,
    /// RNG seed for kickoffs/respawns. `None` = random; `Some(seed)` =
    /// deterministic replays.
    pub rng_seed: Option<u64>,
}

impl Default for ArenaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl ArenaConfig {
    /// Soccar defaults with matching [`MutatorConfig`].
    pub const DEFAULT: Self = Self {
        game_mode: GameMode::Soccar,
        mutators: MutatorConfig::new(GameMode::Soccar),
        mem_weight_mode: ArenaMemWeightMode::Heavy,
        min_pos: Vec3A::new(-5600., -6000., 0.),
        max_pos: Vec3A::new(5600., 6000., 2200.),
        max_aabb_len: 370.,
        no_ball_rot: false,
        custom_boost_pads: None,
        rng_seed: None,
    };

    /// Config for a game mode with that mode's default mutators.
    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self {
            game_mode,
            mutators: MutatorConfig::new(game_mode),
            ..Self::DEFAULT
        }
    }

    /// Override the physics/rule mutators.
    #[must_use]
    pub fn with_mutators(mut self, mutators: MutatorConfig) -> Self {
        self.mutators = mutators;
        self
    }

    /// Override the broadphase memory/performance tradeoff.
    #[must_use]
    pub fn with_mem_weight_mode(mut self, mem_weight_mode: ArenaMemWeightMode) -> Self {
        self.mem_weight_mode = mem_weight_mode;
        self
    }

    /// Override the broadphase world minimum (uu).
    #[must_use]
    pub fn with_min_pos(mut self, min_pos: Vec3A) -> Self {
        self.min_pos = min_pos;
        self
    }

    /// Override the broadphase world maximum (uu).
    #[must_use]
    pub fn with_max_pos(mut self, max_pos: Vec3A) -> Self {
        self.max_pos = max_pos;
        self
    }

    /// Override the broadphase BVH leaf size (uu).
    #[must_use]
    pub fn with_max_aabb_len(mut self, max_aabb_len: f32) -> Self {
        self.max_aabb_len = max_aabb_len;
        self
    }

    /// Disable ball angular motion.
    #[must_use]
    pub fn with_no_ball_rot(mut self, no_ball_rot: bool) -> Self {
        self.no_ball_rot = no_ball_rot;
        self
    }

    /// Replace the mode's default boost-pad layout.
    #[must_use]
    pub fn with_custom_boost_pads(mut self, custom_boost_pads: Vec<BoostPadConfig>) -> Self {
        self.custom_boost_pads = Some(custom_boost_pads);
        self
    }

    /// Seed kickoff/respawn RNG for deterministic replays.
    #[must_use]
    pub fn with_rng_seed(mut self, rng_seed: u64) -> Self {
        self.rng_seed = Some(rng_seed);
        self
    }
}
