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
    car_idx: usize,
    winners: &'a [Option<usize>],
    ordered_arbitration: bool,
}

struct MembershipProcessor<'a> {
    all_pads: &'a mut [BoostPad],
    car_idx: usize,
    car_pos: glam::Vec3A,
}

impl bvh::ProcessNode for MembershipProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        let pad = &mut self.all_pads[pad_idx];
        if overlaps_values(pad.config().pos, pad.cyl_radius, self.car_pos)
            && !pad.touching_car_idxs.contains(&self.car_idx)
        {
            pad.touching_car_idxs.push(self.car_idx);
        }
    }
}

fn overlaps_values(pad_pos: glam::Vec3A, radius: f32, pos: glam::Vec3A) -> bool {
    let dist_sq_2d = pad_pos.truncate().distance_squared(pos.truncate());
    dist_sq_2d < radius.powi(2) && (pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT
}

impl bvh::ProcessNode for BoostPadProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        if self.pad_idx.is_some() {
            return; // Already found a pad to give boost from, no need to check more
        }

        let pad = &mut self.all_pads[pad_idx];

        if self.ordered_arbitration && self.winners[pad_idx] != Some(self.car_idx) {
            return;
        }

        if let Some(last_give_tick_count) = pad.gave_boost_tick_count
            && ((self.tick_count as i64 - last_give_tick_count) as f32 * TICK_TIME)
                < pad.max_cooldown
        {
            return;
        }

        if overlaps_values(pad.config().pos, pad.cyl_radius, self.car_state.pos) {
            // Give boost
            self.car_state.boost = (self.car_state.boost + pad.boost_amount)
                .min(self.mutator_config.car_max_boost_amount);
            pad.gave_boost_tick_count = Some(self.tick_count as i64);
            self.pad_idx = Some(pad_idx);
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    pub bvh_tree: bvh::Tree,
    pub all_pads: Vec<BoostPad>,
    pub max_pad_z: f32,
    winners: Vec<Option<usize>>,
    ordered_arbitration: bool,
}

impl BoostPadGrid {
    pub fn new(
        pad_configs: &[BoostPadConfig],
        mutator_config: &MutatorConfig,
        ordered_arbitration: bool,
    ) -> Self {
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

        let mut aabb_nodes = Vec::new();
        for (i, pad) in all_pads.iter().enumerate() {
            let node = bvh::Node {
                aabb: pad.aabb(),
                node_type: bvh::BvhNodeType::Leaf { leaf_idx: i },
            };
            aabb_nodes.push(node);
        }

        let bvh_tree = bvh::Tree::build(all_aabb, &mut aabb_nodes);

        let pad_count = all_pads.len();
        Self {
            bvh_tree,
            all_pads,
            max_pad_z: all_aabb.max.z,
            winners: vec![None; pad_count],
            ordered_arbitration,
        }
    }

    pub fn reset(&mut self) {
        for pad in &mut self.all_pads {
            pad.reset();
        }
        self.winners.fill(None);
    }

    pub fn update_membership(&mut self, cars: &[crate::sim::car::Car], max_boost: f32) {
        for pad in &mut self.all_pads {
            let pad_pos = pad.config().pos;
            let radius = pad.cyl_radius;
            pad.touching_car_idxs.retain(|&idx| {
                cars.get(idx).is_some_and(|car| {
                    !car.state.is_demoed && overlaps_values(pad_pos, radius, car.state.pos)
                })
            });
        }

        for (car_idx, car) in cars.iter().enumerate() {
            if car.state.is_demoed || car.state.pos.z > self.max_pad_z {
                continue;
            }

            let car_aabb = Aabb::new(car.state.pos, car.state.pos);
            let mut processor = MembershipProcessor {
                all_pads: &mut self.all_pads,
                car_idx,
                car_pos: car.state.pos,
            };

            self.bvh_tree
                .report_aabb_overlapping_node(&mut processor, &car_aabb);
        }
        self.refresh_winners(cars, max_boost);
    }

    pub fn refresh_winners(&mut self, cars: &[crate::sim::car::Car], max_boost: f32) {
        for (pad_idx, pad) in self.all_pads.iter().enumerate() {
            self.winners[pad_idx] = pad.touching_car_idxs.iter().copied().find(|&idx| {
                cars.get(idx).is_some_and(|car| {
                    !car.state.is_demoed
                        && matches!(
                            car.state.boost.partial_cmp(&max_boost),
                            Some(std::cmp::Ordering::Less) | None
                        )
                })
            });
        }
    }

    /// If boost was collected, returns the pad index
    pub(crate) fn maybe_give_car_boost(
        &mut self,
        car_state: &mut CarState,
        mutator_config: &MutatorConfig,
        tick_count: u64,
        car_idx: usize,
    ) -> Option<usize> {
        if car_state.is_demoed {
            return None; // The stored position remains after demolition.
        }

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
            car_idx,
            winners: &self.winners,
            ordered_arbitration: self.ordered_arbitration,
        };
        self.bvh_tree
            .report_aabb_overlapping_node(&mut pad_processor, &car_center_aabb);

        pad_processor.pad_idx
    }
}
