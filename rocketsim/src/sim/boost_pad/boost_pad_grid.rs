use std::sync::Arc;

use glam::Vec3A;

use super::{BoostPad, BoostPadGeometry};
use crate::{
    BoostPadConfig, BoostPadState, Car, CarState, GameMode, MutatorConfig,
    consts::{TICK_RATE, TICK_TIME, boost_pads},
    shared::{Aabb, bvh},
};

/// Stored pickup tick for a pad that never gave boost.
const NO_PICKUP_TICK: i64 = i64::MIN;
/// Cached winner for a pad that has no winner.
const NO_WINNER: usize = usize::MAX;

struct MembershipProcessor<'a> {
    pads: &'a [BoostPad],
    touching_car_idxs: &'a mut [Vec<usize>],
    touching_pad_by_car: &'a mut [Option<usize>],
    car_idx: usize,
    car_pos: Vec3A,
}

impl bvh::ProcessNode for MembershipProcessor<'_> {
    fn process_node(&mut self, pad_idx: usize) {
        if self.touching_pad_by_car[self.car_idx].is_some() {
            return;
        }

        let pad = &self.pads[pad_idx];
        if overlaps_values(pad.config().pos, pad.cyl_radius(), self.car_pos) {
            let members = &mut self.touching_car_idxs[pad_idx];
            if !members.contains(&self.car_idx) {
                members.push(self.car_idx);
            }
            self.touching_pad_by_car[self.car_idx] = Some(pad_idx);
        }
    }
}

fn overlaps_values(pad_pos: Vec3A, radius: f32, pos: Vec3A) -> bool {
    let dist_sq_2d = pad_pos.truncate().distance_squared(pos.truncate());
    dist_sq_2d < radius.powi(2) && (pos.z - pad_pos.z).abs() <= boost_pads::CYL_HEIGHT
}

fn can_receive_boost(car: &Car, max_boost: f32) -> bool {
    !car.state.is_demoed
        && matches!(
            car.state.boost.partial_cmp(&max_boost),
            Some(std::cmp::Ordering::Less) | None
        )
}

fn hitbox_overlaps_pad_box(pad_pos: Vec3A, box_radius: f32, car_center: Vec3A, car: &Car) -> bool {
    let rot = car.state.phys.rot_mat;
    let config = car.info.config;
    let center = car_center + rot * config.hitbox_pos_offset;
    let car_axes = [rot.x_axis, rot.y_axis, rot.z_axis];
    let car_half = config.hitbox_size * 0.5;
    let pad_half = Vec3A::new(box_radius, box_radius, boost_pads::BOX_HEIGHT);
    let delta = center - pad_pos;
    let world_axes = [Vec3A::X, Vec3A::Y, Vec3A::Z];

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

        let pad_radius = pad_half.dot(Vec3A::new(
            world_axes[0].dot(axis).abs(),
            world_axes[1].dot(axis).abs(),
            world_axes[2].dot(axis).abs(),
        ));
        let car_radius = car_half.dot(Vec3A::new(
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

/// Per-arena boost-pad state.
///
/// The immutable layout and BVH are shared through [`BoostPadGeometry`].
/// This type stores only mutable state: pickup ticks, ordered membership,
/// cached winners, and cooldowns and amounts for both pad classes.
#[derive(Debug, Clone)]
pub(crate) struct BoostPadGrid {
    geometry: Arc<BoostPadGeometry>,
    gave_boost_tick_counts: Box<[i64]>,
    touching_car_idxs: Box<[Vec<usize>]>,
    winners: Box<[usize]>,
    touching_pad_by_car: Vec<Option<usize>>,
    /// `[small, big]` cooldown in seconds.
    max_cooldown: [f32; 2],
    /// `[small, big]` boost amount.
    boost_amount: [f32; 2],
}

impl BoostPadGrid {
    /// Build a grid from a custom pad layout.
    ///
    /// # Panics
    ///
    /// Panics if `pad_configs` is empty.
    pub fn new(pad_configs: &[BoostPadConfig], mutator_config: &MutatorConfig) -> Self {
        Self::with_geometry(
            Arc::new(BoostPadGeometry::from_configs(pad_configs)),
            mutator_config,
        )
    }

    /// Build a grid on the shared default layout for `game_mode`.
    pub fn new_default(game_mode: GameMode, mutator_config: &MutatorConfig) -> Self {
        Self::with_geometry(BoostPadGeometry::default_for(game_mode), mutator_config)
    }

    fn with_geometry(geometry: Arc<BoostPadGeometry>, mutator_config: &MutatorConfig) -> Self {
        let pad_count = geometry.num_pads();

        Self {
            geometry,
            gave_boost_tick_counts: vec![NO_PICKUP_TICK; pad_count].into_boxed_slice(),
            touching_car_idxs: (0..pad_count).map(|_| Vec::new()).collect(),
            winners: vec![NO_WINNER; pad_count].into_boxed_slice(),
            touching_pad_by_car: Vec::new(),
            max_cooldown: [
                mutator_config.boost_pad_cooldown_small,
                mutator_config.boost_pad_cooldown_big,
            ],
            boost_amount: [
                mutator_config.boost_pad_amount_small,
                mutator_config.boost_pad_amount_big,
            ],
        }
    }

    pub fn pads(&self) -> &[BoostPad] {
        self.geometry.pads()
    }

    pub fn num_pads(&self) -> usize {
        self.geometry.num_pads()
    }

    /// Last tick that gave boost, or `None` when the pad never gave boost.
    pub fn gave_boost_tick_count(&self, pad_idx: usize) -> Option<i64> {
        let tick = self.gave_boost_tick_counts[pad_idx];
        (tick != NO_PICKUP_TICK).then_some(tick)
    }

    pub fn touching_car_idxs(&self, pad_idx: usize) -> &[usize] {
        &self.touching_car_idxs[pad_idx]
    }

    pub fn max_cooldown(&self, pad_idx: usize) -> f32 {
        self.max_cooldown[self.geometry.pads()[pad_idx].class_index()]
    }

    /// Apply runtime cooldown and amount mutators to this arena only.
    pub fn set_mutators(&mut self, mutators: &MutatorConfig) {
        self.max_cooldown = [
            mutators.boost_pad_cooldown_small,
            mutators.boost_pad_cooldown_big,
        ];
        self.boost_amount = [
            mutators.boost_pad_amount_small,
            mutators.boost_pad_amount_big,
        ];
    }

    pub fn reset(&mut self) {
        self.gave_boost_tick_counts.fill(NO_PICKUP_TICK);
        for members in &mut self.touching_car_idxs {
            members.clear();
        }
        self.winners.fill(NO_WINNER);
        self.touching_pad_by_car.fill(None);
    }

    pub(crate) fn remove_car(&mut self, car_idx: usize, cars: &[Car], max_boost: f32) {
        for members in &mut self.touching_car_idxs {
            members.retain_mut(|idx| {
                if *idx == car_idx {
                    return false;
                }
                if *idx > car_idx {
                    *idx -= 1;
                }
                true
            });
        }
        // Cars added since the last membership update have no cache entry yet.
        if car_idx < self.touching_pad_by_car.len() {
            self.touching_pad_by_car.remove(car_idx);
        }
        self.refresh_winners(cars, max_boost);
    }

    pub(crate) fn remove_cars(&mut self) {
        for members in &mut self.touching_car_idxs {
            members.clear();
        }
        self.winners.fill(NO_WINNER);
        self.touching_pad_by_car.clear();
    }

    /// Update pad membership for the current cars.
    ///
    /// Returns `true` when at least one car touches a pad. When the result is
    /// `false`, no car can collect boost this tick, so the caller can skip the
    /// per-car award check.
    pub fn update_membership(&mut self, cars: &[Car], max_boost: f32) -> bool {
        self.touching_pad_by_car.resize(cars.len(), None);
        self.touching_pad_by_car.fill(None);

        // Tests the car's position halfway through the tick,
        // so a pickup can land on this tick or the next.
        // Keep the native pad volumes, but probe at the same phase.
        let probe = |car: &Car| car.state.pos - car.state.vel * (0.5 * TICK_TIME);

        let pads = self.geometry.pads();
        let touching_car_idxs = &mut self.touching_car_idxs;
        let touching_pad_by_car = &mut self.touching_pad_by_car;

        // Pad order resolves duplicate restored membership deterministically.
        for (pad_idx, members) in touching_car_idxs.iter_mut().enumerate() {
            // A pad without retained members has no entry to validate.
            if members.is_empty() {
                continue;
            }

            let pad = &pads[pad_idx];
            let pad_pos = pad.config().pos;
            let radius = pad.cyl_radius();
            let box_radius = pad.box_radius();
            members.retain(|&idx| {
                cars.get(idx).is_some_and(|car| {
                    if car.state.is_demoed || touching_pad_by_car[idx].is_some() {
                        return false;
                    }

                    let car_pos = probe(car);
                    if overlaps_values(pad_pos, radius, car_pos)
                        || hitbox_overlaps_pad_box(pad_pos, box_radius, car_pos, car)
                    {
                        touching_pad_by_car[idx] = Some(pad_idx);
                        true
                    } else {
                        false
                    }
                })
            });
        }

        let max_pad_z = self.geometry.max_pad_z();
        for (car_idx, car) in cars.iter().enumerate() {
            if car.state.is_demoed || car.state.pos.z > max_pad_z {
                continue;
            }

            // One car can touch one pad. Retention makes discovery unnecessary.
            if touching_pad_by_car[car_idx].is_some() {
                continue;
            }

            let car_pos = probe(car);
            let car_aabb = Aabb::new(car_pos, car_pos);
            let mut processor = MembershipProcessor {
                pads,
                touching_car_idxs: &mut *touching_car_idxs,
                touching_pad_by_car: &mut *touching_pad_by_car,
                car_idx,
                car_pos,
            };

            self.geometry
                .bvh_tree()
                .report_aabb_overlapping_node(&mut processor, &car_aabb);
        }
        self.refresh_winners(cars, max_boost);

        self.touching_pad_by_car
            .iter()
            .any(|pad_idx| pad_idx.is_some())
    }

    pub fn refresh_winners(&mut self, cars: &[Car], max_boost: f32) {
        for pad_idx in 0..self.touching_car_idxs.len() {
            let members = &self.touching_car_idxs[pad_idx];
            let winner = if members.is_empty() {
                // An empty pad has no winner. Clear the cached entry so a
                // stale winner cannot award boost.
                NO_WINNER
            } else {
                members
                    .iter()
                    .copied()
                    .find(|&idx| {
                        cars.get(idx)
                            .is_some_and(|car| can_receive_boost(car, max_boost))
                    })
                    .unwrap_or(NO_WINNER)
            };
            self.winners[pad_idx] = winner;
        }
    }

    pub fn refresh_winner(&mut self, pad_idx: usize, cars: &[Car], max_boost: f32) {
        self.winners[pad_idx] = self.touching_car_idxs[pad_idx]
            .iter()
            .copied()
            .find(|&idx| {
                cars.get(idx)
                    .is_some_and(|car| can_receive_boost(car, max_boost))
            })
            .unwrap_or(NO_WINNER);
    }

    /// Restore pad cooldown and ordered membership from a snapshot.
    ///
    /// Membership uses the same de-duplication and order rules as
    /// [`crate::Arena::set_boost_pad_state`].
    pub(crate) fn set_pad_state(
        &mut self,
        pad_idx: usize,
        state: &BoostPadState,
        car_count: usize,
        tick_count: u64,
    ) {
        let class = self.geometry.pads()[pad_idx].class_index();
        let members = &mut self.touching_car_idxs[pad_idx];
        members.clear();

        for car_idx in state.touching_car_idxs.iter().copied() {
            if car_idx < car_count && !members.contains(&car_idx) {
                members.push(car_idx);
            }
        }

        if state.cooldown > 0.0 {
            let max_cooldown = self.max_cooldown[class];
            let time_since_pickup = (max_cooldown - state.cooldown).max(0.0);
            let ticks_since_pickup = (time_since_pickup * TICK_RATE).round() as i64;
            self.gave_boost_tick_counts[pad_idx] = tick_count as i64 - ticks_since_pickup;
        } else {
            self.gave_boost_tick_counts[pad_idx] = NO_PICKUP_TICK;
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

        let pad_idx = self.touching_pad_by_car.get(car_idx).copied().flatten()?;
        if self.winners[pad_idx] != car_idx {
            return None;
        }

        let class = self.geometry.pads()[pad_idx].class_index();
        let last_give_tick_count = self.gave_boost_tick_counts[pad_idx];
        if last_give_tick_count != NO_PICKUP_TICK
            && ((tick_count as i64 - last_give_tick_count) as f32 * TICK_TIME)
                < self.max_cooldown[class]
        {
            return None;
        }

        car_state.boost =
            (car_state.boost + self.boost_amount[class]).min(mutator_config.car_max_boost_amount);
        self.gave_boost_tick_counts[pad_idx] = tick_count as i64;
        Some(pad_idx)
    }
}
