use std::sync::{Arc, OnceLock};

use glam::Vec3A;

use crate::{
    BoostPadConfig, GameMode,
    consts::boost_pads,
    shared::{Aabb, bvh},
};

/// Immutable geometry of one boost pad.
///
/// The default layouts share one `BoostPadGeometry` through an `Arc`. Do not
/// store mutable pad state here. Cooldowns and membership belong to
/// [`BoostPadGrid`](super::BoostPadGrid).
#[derive(Debug, Clone, Copy)]
pub(crate) struct BoostPad {
    config: BoostPadConfig,
    box_radius: f32,
    cyl_radius: f32,
    aabb: Aabb,
}

impl BoostPad {
    fn new(config: BoostPadConfig) -> Self {
        let box_radius = if config.is_big {
            boost_pads::BOX_RAD_BIG
        } else {
            boost_pads::BOX_RAD_SMALL
        };

        let cyl_radius = if config.is_big {
            boost_pads::CYL_RAD_BIG
        } else {
            boost_pads::CYL_RAD_SMALL
        };

        let extent = Vec3A::new(cyl_radius, cyl_radius, boost_pads::CYL_HEIGHT);
        let aabb = Aabb::new(config.pos - extent, config.pos + extent);

        Self {
            config,
            box_radius,
            cyl_radius,
            aabb,
        }
    }

    pub const fn config(&self) -> &BoostPadConfig {
        &self.config
    }

    pub const fn aabb(&self) -> Aabb {
        self.aabb
    }

    pub const fn box_radius(&self) -> f32 {
        self.box_radius
    }

    pub const fn cyl_radius(&self) -> f32 {
        self.cyl_radius
    }

    /// Size class index: `0` for small pads, `1` for big pads.
    pub const fn class_index(&self) -> usize {
        self.config.is_big as usize
    }
}

/// Immutable pad layout for one arena shape: ordered pads and their BVH.
///
/// Pad order is Y, then X, as in RLBot and RLGym.
/// Build the default layout once per process. Equal default arenas then share
/// the same `Arc`. Custom layouts build one private value for their arena.
#[derive(Debug)]
pub(crate) struct BoostPadGeometry {
    pads: Box<[BoostPad]>,
    bvh_tree: bvh::Tree,
}

impl BoostPadGeometry {
    /// Build a geometry value from a custom layout.
    ///
    /// # Panics
    ///
    /// Panics if `pad_configs` is empty.
    pub fn from_configs(pad_configs: &[BoostPadConfig]) -> Self {
        assert!(!pad_configs.is_empty());

        let mut pads: Vec<BoostPad> = pad_configs.iter().copied().map(BoostPad::new).collect();

        // Sort them to match RLBot/RLGym ordering
        pads.sort_by(|a, b| {
            let a_pos = a.config.pos;
            let b_pos = b.config.pos;
            match a_pos.y.total_cmp(&b_pos.y) {
                std::cmp::Ordering::Equal => a_pos.x.total_cmp(&b_pos.x),
                other => other,
            }
        });

        let all_aabb = {
            let mut all_aabb_accum: Option<Aabb> = None;

            for pad in &pads {
                let pad_aabb = pad.aabb();
                if let Some(all_aabb) = all_aabb_accum {
                    all_aabb_accum = Some(all_aabb.combine(&pad_aabb));
                } else {
                    all_aabb_accum = Some(pad_aabb);
                }
            }
            all_aabb_accum.unwrap()
        };

        let mut aabb_nodes = Vec::with_capacity(pads.len());
        for (i, pad) in pads.iter().enumerate() {
            let node = bvh::Node {
                aabb: pad.aabb(),
                node_type: bvh::BvhNodeType::Leaf { leaf_idx: i },
            };
            aabb_nodes.push(node);
        }

        let bvh_tree = bvh::Tree::build(all_aabb, &mut aabb_nodes);

        Self {
            pads: pads.into_boxed_slice(),
            bvh_tree,
        }
    }

    /// Return the shared default geometry for `game_mode`.
    ///
    /// Soccar, Heatseeker, and Snowday use the Soccar positions. Hoops has its
    /// own positions. These two layouts are the only process-wide entries.
    pub fn default_for(game_mode: GameMode) -> Arc<Self> {
        static SOCCAR: OnceLock<Arc<BoostPadGeometry>> = OnceLock::new();
        static HOOPS: OnceLock<Arc<BoostPadGeometry>> = OnceLock::new();

        let cell = if game_mode == GameMode::Hoops {
            &HOOPS
        } else {
            &SOCCAR
        };

        Arc::clone(
            cell.get_or_init(|| Arc::new(Self::from_configs(&default_pad_configs(game_mode)))),
        )
    }

    pub const fn pads(&self) -> &[BoostPad] {
        &self.pads
    }

    pub const fn bvh_tree(&self) -> &bvh::Tree {
        &self.bvh_tree
    }

    pub const fn num_pads(&self) -> usize {
        self.pads.len()
    }

    pub fn max_pad_z(&self) -> f32 {
        self.bvh_tree.aabb.max.z
    }
}

fn default_pad_configs(game_mode: GameMode) -> Vec<BoostPadConfig> {
    let small_pad_locs = boost_pads::get_locations(game_mode, false);
    let big_pad_locs = boost_pads::get_locations(game_mode, true);
    let mut boost_pad_configs = Vec::with_capacity(small_pad_locs.len() + big_pad_locs.len());

    for small_pos in small_pad_locs {
        boost_pad_configs.push(BoostPadConfig {
            pos: *small_pos,
            is_big: false,
        });
    }

    for big_pos in big_pad_locs {
        boost_pad_configs.push(BoostPadConfig {
            pos: *big_pos,
            is_big: true,
        });
    }

    boost_pad_configs
}
