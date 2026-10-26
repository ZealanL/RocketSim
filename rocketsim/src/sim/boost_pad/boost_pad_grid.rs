use crate::{
    BoostPad, BoostPadConfig, CarState, MutatorConfig,
    consts::{TICK_TIME, boost_pads},
    shared::{Aabb, bvh},
};

struct MembershipProcessor<'a> {
    all_pads: &'a mut [BoostPad],
    touching_pad_by_car: &'a mut [Option<usize>],
    car_idx: usize,
    car_pos: glam::Vec3A,
}

impl bvh::ProcessNode for MembershipProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        if self.touching_pad_by_car[self.car_idx].is_some() {
            return;
        }

        let pad = &mut self.all_pads[pad_idx];
        if overlaps_values(pad.config().pos, pad.cyl_radius, self.car_pos) {
            if !pad.touching_car_idxs.contains(&self.car_idx) {
                pad.touching_car_idxs.push(self.car_idx);
            }
            self.touching_pad_by_car[self.car_idx] = Some(pad_idx);
        }
    }
}

fn overlaps_values(pad_pos: glam::Vec3A, radius: f32, pos: glam::Vec3A) -> bool {
    let dist_sq_2d = pad_pos.truncate().distance_squared(pos.truncate());
    dist_sq_2d < radius.powi(2) && (pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT
}

fn can_receive_boost(car: &crate::sim::car::Car, max_boost: f32) -> bool {
    !car.state.is_demoed
        && matches!(
            car.state.boost.partial_cmp(&max_boost),
            Some(std::cmp::Ordering::Less) | None
        )
}

fn hitbox_overlaps_pad_box(
    pad_pos: glam::Vec3A,
    box_radius: f32,
    car: &crate::sim::car::Car,
) -> bool {
    let rot = car.state.phys.rot_mat;
    let config = car.info.config;
    let center = car.state.pos + rot * config.hitbox_pos_offset;
    let car_axes = [rot.x_axis, rot.y_axis, rot.z_axis];
    let car_half = config.hitbox_size * 0.5;
    let pad_half = glam::Vec3A::new(box_radius, box_radius, boost_pads::BOX_HEIGHT);
    let delta = center - pad_pos;
    let world_axes = [glam::Vec3A::X, glam::Vec3A::Y, glam::Vec3A::Z];

    for axis in world_axes
        .into_iter()
        .chain(car_axes)
        .chain(car_axes.into_iter().flat_map(|car_axis| {
            world_axes
                .into_iter()
                .map(move |world_axis| car_axis.cross(world_axis))
        }))
    {
        if axis.length_squared() <= f32::EPSILON {
            continue;
        }

        let pad_radius = pad_half.dot(glam::Vec3A::new(
            world_axes[0].dot(axis).abs(),
            world_axes[1].dot(axis).abs(),
            world_axes[2].dot(axis).abs(),
        ));
        let car_radius = car_half.dot(glam::Vec3A::new(
            car_axes[0].dot(axis).abs(),
            car_axes[1].dot(axis).abs(),
            car_axes[2].dot(axis).abs(),
        ));
        if delta.dot(axis).abs() > pad_radius + car_radius {
            return false;
        }
    }

    true
}

#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    pub bvh_tree: bvh::Tree,
    pub all_pads: Vec<BoostPad>,
    pub max_pad_z: f32,
    winners: Vec<Option<usize>>,
    touching_pad_by_car: Vec<Option<usize>>,
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
            touching_pad_by_car: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        for pad in &mut self.all_pads {
            pad.reset();
        }
        self.winners.fill(None);
        self.touching_pad_by_car.fill(None);
    }

    pub fn update_membership(&mut self, cars: &[crate::sim::car::Car], max_boost: f32) {
        self.touching_pad_by_car.resize(cars.len(), None);
        self.touching_pad_by_car.fill(None);
        let touching_pad_by_car = &mut self.touching_pad_by_car;
        // Pad order resolves duplicate restored membership deterministically.
        for (pad_idx, pad) in self.all_pads.iter_mut().enumerate() {
            let pad_pos = pad.config().pos;
            let radius = pad.cyl_radius;
            let box_radius = pad.box_radius;
            pad.touching_car_idxs.retain(|&idx| {
                cars.get(idx).is_some_and(|car| {
                    if car.state.is_demoed || touching_pad_by_car[idx].is_some() {
                        return false;
                    }
                    if overlaps_values(pad_pos, radius, car.state.pos)
                        || hitbox_overlaps_pad_box(pad_pos, box_radius, car)
                    {
                        touching_pad_by_car[idx] = Some(pad_idx);
                        true
                    } else {
                        false
                    }
                })
            });
        }

        for (car_idx, car) in cars.iter().enumerate() {
            if car.state.is_demoed || car.state.pos.z > self.max_pad_z {
                continue;
            }
            // One car can touch one pad. Retention makes discovery unnecessary.
            if self.touching_pad_by_car[car_idx].is_some() {
                continue;
            }

            let car_aabb = Aabb::new(car.state.pos, car.state.pos);
            let mut processor = MembershipProcessor {
                all_pads: &mut self.all_pads,
                touching_pad_by_car: &mut self.touching_pad_by_car,
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
            let winner = pad.touching_car_idxs.iter().copied().find(|&idx| {
                cars.get(idx)
                    .is_some_and(|car| can_receive_boost(car, max_boost))
            });
            self.winners[pad_idx] = winner;
        }
    }

    pub fn refresh_winner(
        &mut self,
        pad_idx: usize,
        cars: &[crate::sim::car::Car],
        max_boost: f32,
    ) {
        self.winners[pad_idx] = self.all_pads[pad_idx]
            .touching_car_idxs
            .iter()
            .copied()
            .find(|&idx| {
                cars.get(idx)
                    .is_some_and(|car| can_receive_boost(car, max_boost))
            });
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

        let pad_idx = self.touching_pad_by_car.get(car_idx).copied().flatten()?;
        if self.winners[pad_idx] != Some(car_idx) {
            return None;
        }

        let pad = &mut self.all_pads[pad_idx];
        if let Some(last_give_tick_count) = pad.gave_boost_tick_count
            && ((tick_count as i64 - last_give_tick_count) as f32 * TICK_TIME) < pad.max_cooldown
        {
            return None;
        }

        car_state.boost =
            (car_state.boost + pad.boost_amount).min(mutator_config.car_max_boost_amount);
        pad.gave_boost_tick_count = Some(tick_count as i64);
        Some(pad_idx)
    }
}
