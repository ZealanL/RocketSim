use crate::{
    BoostPad, BoostPadConfig, CarState, MutatorConfig,
    consts::{TICK_TIME, boost_pads},
    shared::{Aabb, bvh},
};

pub struct BoostPadProcessor<'a> {
    all_pads: &'a mut [BoostPad],
    car_state: &'a mut CarState,
    mutator_config: &'a MutatorConfig,
    tick_count: u64,
    pad_idx: Option<usize>,
}

impl bvh::ProcessNode for BoostPadProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        if self.pad_idx.is_some() {
            return; // Already found a pad to give boost from, no need to check more
        }

        let pad = &mut self.all_pads[pad_idx];

        if let Some(last_give_tick_count) = pad.gave_boost_tick_count
            && ((self.tick_count - last_give_tick_count) as f32 * TICK_TIME) < pad.max_cooldown
        {
            return;
        }

        // Check if car origin is inside the cylinder hitbox
        let pad_pos = pad.config().pos;
        let dist_sq_2d = pad_pos
            .truncate()
            .distance_squared(self.car_state.pos.truncate());
        let overlapping = dist_sq_2d < pad.cyl_radius.powi(2)
            && (self.car_state.pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT;
        if overlapping {
            // Give boost
            self.car_state.boost = (self.car_state.boost + pad.boost_amount)
                .min(self.mutator_config.car_max_boost_amount);
            pad.gave_boost_tick_count = Some(self.tick_count);
            self.pad_idx = Some(pad_idx);
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    pub bvh_tree: bvh::Tree,
    pub all_pads: Vec<BoostPad>,
    pub max_pad_z: f32,
}

impl BoostPadGrid {
    pub fn new(pad_configs: &[BoostPadConfig], mutator_config: &MutatorConfig) -> Self {
        assert!(!pad_configs.is_empty());

        let mut all_pads: Vec<BoostPad> = pad_configs
            .iter()
            .map(|&pad_config| BoostPad::new(pad_config, mutator_config))
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

    /// If boost was collected, returns the pad index
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        mutator_config: &MutatorConfig,
        tick_count: u64,
    ) -> Option<usize> {
        if car_state.boost >= mutator_config.car_max_boost_amount {
            return None; // Already full on boost
        }

        if car_state.pos.z > self.max_pad_z {
            return None; // Can't possibly overlap with a boost pad
        }

        let car_center_aabb = Aabb::new(car_state.pos, car_state.pos);

        let mut pad_processor = BoostPadProcessor {
            all_pads: &mut self.all_pads,
            car_state,
            mutator_config,
            tick_count,
            pad_idx: None,
        };
        self.bvh_tree
            .report_aabb_overlapping_node(&mut pad_processor, &car_center_aabb);

        pad_processor.pad_idx
    }
}
