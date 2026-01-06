use crate::consts::TICK_TIME;
use crate::shared::bvh::SimpleNodeProcessor;
use crate::shared::{Aabb, bvh};
use crate::{BoostPad, BoostPadConfig, CarState, MutatorConfig, consts::boost_pads};

#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    pub bvh_tree: bvh::Tree,
    pub all_pads: Vec<BoostPad>,
    pub max_pad_z: f32,
}

impl BoostPadGrid {
    pub fn new(pad_configs: &Vec<BoostPadConfig>, mutator_config: &MutatorConfig) -> Self {
        assert!(!pad_configs.is_empty());

        let mut all_pads: Vec<BoostPad> = pad_configs
            .clone()
            .into_iter()
            .map(|pad_config| BoostPad::new(pad_config, mutator_config))
            .collect();

        // Sort them to match RLBot/RLGym ordering
        all_pads.sort_by(|a, b| {
            let a_pos = a.config.pos;
            let b_pos = b.config.pos;
            match a_pos.y.total_cmp(&b_pos.y) {
                std::cmp::Ordering::Equal => a_pos.x.total_cmp(&b_pos.x),
                other => other,
            }
        });

        let all_aabb = {
            let mut all_aabb_accum: Option<Aabb> = None;

            for pad in &all_pads {
                let pad_aabb = pad.aabb();
                if let Some(all_aabb) = all_aabb_accum {
                    all_aabb_accum = Some(all_aabb.combine(&pad_aabb));
                } else {
                    all_aabb_accum = Some(pad_aabb);
                }
            }
            all_aabb_accum.unwrap()
        };

        let mut bvh_tree = bvh::Tree::new(all_aabb, pad_configs.len());

        let mut aabb_nodes = Vec::new();
        for (i, pad) in all_pads.iter().enumerate() {
            let node = bvh::Node {
                aabb: pad.aabb(),
                node_type: bvh::BvhNodeType::Leaf { leaf_idx: i },
            };
            aabb_nodes.push(node);
        }

        let num_nodes = aabb_nodes.len();
        bvh_tree.build_tree(&mut aabb_nodes, 0, num_nodes);

        Self {
            bvh_tree,
            all_pads,
            max_pad_z: all_aabb.max.z,
        }
    }

    pub fn reset(&mut self) {
        for pad in &mut self.all_pads {
            pad.reset();
        }
    }

    /// Returns true if boost was given
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        mutator_config: &MutatorConfig,
        tick_count: u64,
    ) {
        if car_state.boost >= mutator_config.car_max_boost_amount {
            return; // Already full on boost
        }

        if car_state.pos.z > self.max_pad_z {
            return; // Can't possibly overlap with a boost pad
        }

        let car_center_aabb = Aabb::new(car_state.pos, car_state.pos);
        let mut node_processor = SimpleNodeProcessor::new();
        self.bvh_tree
            .report_aabb_overlapping_node(&mut node_processor, &car_center_aabb);

        for pad_idx in node_processor.leaf_indices {
            let pad = &mut self.all_pads[pad_idx];

            if let Some(last_give_tick_count) = pad.gave_boost_tick_count
                && ((tick_count - last_give_tick_count) as f32 * TICK_TIME) < pad.max_cooldown
            {
                continue;
            }

            // Check if car origin is inside the cylinder hitbox
            let pad_pos = pad.config().pos;
            let dist_sq_2d = pad_pos
                .truncate()
                .distance_squared(car_state.pos.truncate());
            let overlapping = dist_sq_2d < pad.cyl_radius.powi(2)
                && (car_state.pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT;
            if overlapping {
                // Give boost
                car_state.boost =
                    (car_state.boost + pad.boost_amount).min(mutator_config.car_max_boost_amount);
                pad.gave_boost_tick_count = Some(tick_count);
                return;
            }
        }
    }
}
