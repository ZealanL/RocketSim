use crate::consts::boost_pads;
use crate::shared::Aabb;
use crate::{BoostPadConfig, MutatorConfig};
use glam::Vec3A;

#[derive(Debug, Copy, Clone)]
pub(crate) struct BoostPad {
    pub config: BoostPadConfig,
    pub _box_radius: f32, // TODO: Implement car-locking with box hitbox
    pub cyl_radius: f32,
    pub max_cooldown: f32,
    pub boost_amount: f32,
    pub aabb: Aabb,
    pub gave_boost_tick_count: Option<u64>,
}

impl BoostPad {
    pub fn new(config: BoostPadConfig, mutator_config: &MutatorConfig) -> Self {
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

        let max_cooldown = if config.is_big {
            mutator_config.boost_pad_cooldown_big
        } else {
            mutator_config.boost_pad_cooldown_small
        };

        let boost_amount = if config.is_big {
            mutator_config.boost_pad_amount_big
        } else {
            mutator_config.boost_pad_amount_small
        };

        let extent = Vec3A::new(box_radius, box_radius, boost_pads::CYL_HEIGHT);
        let aabb = Aabb::new(config.pos - extent, config.pos + extent);

        Self {
            config,
            _box_radius: box_radius,
            cyl_radius,
            max_cooldown,
            boost_amount,
            aabb,
            gave_boost_tick_count: None,
        }
    }

    pub const fn reset(&mut self) {
        self.gave_boost_tick_count = None;
    }

    pub const fn config(&self) -> &BoostPadConfig {
        &self.config
    }

    pub const fn aabb(&self) -> Aabb {
        self.aabb
    }
}
