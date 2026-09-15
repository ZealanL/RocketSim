use glam::{Quat, Vec3A};

use super::{
    NUM_WHEELS,
    raycaster::VehicleRaycaster,
    wheel_info::{FrictionCurveInput, WheelInfo},
};
use crate::{
    bullet::{
        collision::broadphase::CollisionFilterGroups,
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{Impulse, RigidBody},
        },
        linear_math::QuatExt,
    },
    sim::UserInfoTypes,
};

pub struct VehicleRL {
    raycaster: VehicleRaycaster,
    chassis_body_idx: usize,
    pub wheels: [WheelInfo; NUM_WHEELS],
}

impl VehicleRL {
    pub const fn new(chassis_body_idx: usize, wheels: [WheelInfo; NUM_WHEELS]) -> Self {
        Self {
            raycaster: VehicleRaycaster::new(CollisionFilterGroups::DropshotFloor as u8),
            chassis_body_idx,
            wheels,
        }
    }

    pub fn get_upwards_dir_from_wheel_contacts(&self, cb: &RigidBody) -> Vec3A {
        let mut sum_contact_dir = Vec3A::ZERO;
        for wheel in &self.wheels {
            if let Some(raycast_info) = wheel.raycast_info.as_ref() {
                sum_contact_dir += raycast_info.contact_normal;
            }
        }

        sum_contact_dir
            .try_normalize()
            .unwrap_or_else(|| cb.get_up_vector())
    }

    /// Refresh wheel raycast records for the sticky gate.
    #[must_use]
    pub(crate) fn refresh_wheel_contacts(
        &mut self,
        collision_world: &DiscreteDynamicsWorld,
        chassis: &RigidBody,
        time_step: f32,
    ) -> bool {
        let chassis_trans = *chassis.get_world_trans();
        let mut sources = [Vec3A::ZERO; NUM_WHEELS];
        let mut targets = [Vec3A::ZERO; NUM_WHEELS];
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(&chassis_trans);
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        let mut front_axle_cache: Option<(f32, Vec3A)> = None;
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            let front = i < 2;
            if let Some(ray_result) = ray_results[i] {
                let steer_angle = wheel.steer_angle;
                let axle_dir = if front {
                    match front_axle_cache {
                        Some((cached_angle, cached_axle)) if cached_angle == steer_angle => {
                            cached_axle
                        }
                        _ => {
                            let axle = Quat::from_axis_angle_simd(
                                chassis_trans.matrix3.z_axis,
                                steer_angle,
                            ) * chassis_trans.matrix3.y_axis;
                            front_axle_cache = Some((steer_angle, axle));
                            axle
                        }
                    }
                } else {
                    chassis_trans.matrix3.y_axis
                };

                wheel.apply_ray_cast(
                    chassis,
                    &chassis_trans,
                    axle_dir,
                    ray_result,
                    time_step,
                    front,
                );
            } else {
                wheel.reset_wheel_suspension();
            }
        }

        self.wheels.iter().any(|wheel| {
            wheel
                .raycast_info
                .as_ref()
                .is_some_and(|info| info.is_in_contact_with_world)
        })
    }

    pub const fn get_num_wheels(&self) -> usize {
        self.wheels.len()
    }

    /// Target I23 hit-body reaction: each wheel with a resolved car
    /// pushback applies the equal-and-opposite impulse to the hit car at
    /// contact minus victim origin. Runs before chassis suspension.
    fn apply_hit_car_pushback(&self, collision_world: &mut DiscreteDynamicsWorld) {
        for wheel in &self.wheels {
            let Some(info) = wheel.raycast_info.as_ref() else {
                continue;
            };
            if wheel.extra_pushback <= 0.0 {
                continue;
            }
            let victim_idx = info.ground_body_idx;
            if victim_idx == self.chassis_body_idx || victim_idx >= collision_world.bodies().len() {
                continue;
            }
            if collision_world.bodies()[victim_idx].user_idx != UserInfoTypes::Car {
                continue;
            }
            let contact_point = info.contact_point;
            let contact_normal = info.contact_normal;
            let full_pushback = wheel.extra_pushback * NUM_WHEELS as f32;
            let victim = &mut collision_world.bodies_mut()[victim_idx];
            if victim.inv_mass == 0.0 {
                continue;
            }
            let victim_offset = contact_point - victim.get_world_trans().translation;
            victim.add_impulse(
                None,
                Impulse::LinearRelPos(-contact_normal * full_pushback, victim_offset),
                true,
                false,
            );
        }
    }

    pub fn update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        time_step: f32,
        handbrake_val: f32,
        real_throttle: f32,
        three_wheels: bool,
    ) {
        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        // The chassis transform and mass are invariant for the whole update:
        // wheel impulses only change velocities, never the transform or mass.
        // Cache them once instead of re-reading them for every wheel.
        let chassis_trans = *chassis.get_world_trans();
        let chassis_translation = chassis_trans.translation;
        let friction_scale = chassis.get_mass() / 3.0;

        let mut sources = [Vec3A::ZERO; NUM_WHEELS];
        let mut targets = [Vec3A::ZERO; NUM_WHEELS];

        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            (sources[i], targets[i]) = wheel.prepare_for_raycast(&chassis_trans);
        }

        let ray_results = self
            .raycaster
            .cast_rays(collision_world, &sources, &targets, chassis);

        // Front wheels normally share one steer angle, so their steered
        // axle is identical. Compute it lazily and reuse it while the
        // steer angle matches (each build needs a sin/cos pair). If a
        // front wheel ever carries a different angle, fall back to its
        // own axle with the original formula.
        let mut front_axle_cache: Option<(f32, Vec3A)> = None;
        let mut num_wheels_in_contact = 0;
        for (i, wheel) in self.wheels.iter_mut().enumerate() {
            if let Some(ray_result) = ray_results[i] {
                num_wheels_in_contact += 1;
                let front = i < 2;
                let steer_angle = wheel.steer_angle;
                let axle_dir = if front {
                    match front_axle_cache {
                        Some((cached_angle, cached_axle)) if cached_angle == steer_angle => {
                            cached_axle
                        }
                        _ => {
                            let axle = Quat::from_axis_angle_simd(
                                chassis_trans.matrix3.z_axis,
                                steer_angle,
                            ) * chassis_trans.matrix3.y_axis;
                            front_axle_cache = Some((steer_angle, axle));
                            axle
                        }
                    }
                } else {
                    chassis_trans.matrix3.y_axis
                };
                wheel.apply_ray_cast(
                    chassis,
                    &chassis_trans,
                    axle_dir,
                    ray_result,
                    time_step,
                    front,
                );
                let is_dynamic_hit = !ray_result.rigid_body.is_static_obj();
                wheel.refresh_friction_curves(
                    chassis,
                    FrictionCurveInput {
                        chassis_translation,
                        contact_normal: ray_result.hit_normal_in_world,
                        handbrake_val,
                        real_throttle,
                        three_wheels,
                        is_dynamic_hit,
                    },
                );
            } else {
                wheel.reset_wheel_suspension();
            }
        }

        if num_wheels_in_contact < 3 {
            for wheel in &mut self.wheels {
                wheel.engine_force /= 4.0;
            }
        }

        // Apply dynamic-body stick before chassis suspension and friction.
        for wheel in &self.wheels {
            let Some(info) = wheel.raycast_info.as_ref() else {
                continue;
            };
            let ground_idx = info.ground_body_idx;
            if info.ground_stick == Vec3A::ZERO
                || ground_idx == self.chassis_body_idx
                || ground_idx >= collision_world.bodies().len()
            {
                continue;
            }

            let ground = &mut collision_world.bodies_mut()[ground_idx];
            if ground.is_static_obj() || ground.inv_mass == 0.0 {
                continue;
            }

            let ground_offset = info.contact_point - ground.get_world_trans().translation;
            ground.add_impulse(
                None,
                Impulse::LinearRelPos(info.ground_stick, ground_offset),
                true,
                false,
            );
        }

        // Target I23 hit-body order: the equal-and-opposite car pushback
        // lands before the chassis suspension in the same tick, beside the
        // stick block. Lever arm is contact minus victim origin, matching
        // the upstream two-body resolve shape.
        self.apply_hit_car_pushback(collision_world);

        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_suspension(chassis, chassis_translation, time_step);
        }

        let chassis = &collision_world.bodies()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.update_friction_impulse(chassis, time_step, friction_scale);
        }

        // note: all suspension MUST be updated before impulses are applied
        let chassis = &mut collision_world.bodies_mut()[self.chassis_body_idx];
        for wheel in &mut self.wheels {
            wheel.apply_friction_impulses(chassis, &chassis_trans, time_step);
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::{Affine3A, Mat3A, Vec3A};

    use super::VehicleRL;
    use crate::{
        bullet::{
            collision::{
                broadphase::GridBroadphase,
                shapes::{
                    box_shape::BoxShape, collision_shape::CollisionShapes,
                    compound_shape::CompoundShape,
                },
            },
            dynamics::{
                discrete_dynamics_world::DiscreteDynamicsWorld,
                rigid_body::{RigidBody, RigidBodyConstructionInfo},
                vehicle::{NUM_WHEELS, wheel_info::RaycastInfo},
            },
        },
        consts::UU_TO_BT,
        sim::UserInfoTypes,
    };

    fn make_body(mass: f32, translation: Vec3A, user_idx: UserInfoTypes) -> RigidBody {
        let child = BoxShape::new(Vec3A::new(1.0, 0.5, 0.3));
        let local_inertia = child.calculate_local_intertia(mass);
        let mut info = RigidBodyConstructionInfo::new(
            mass,
            CollisionShapes::Compound(CompoundShape::new(child, Affine3A::IDENTITY)),
        );
        info.local_inertia = local_inertia;
        info.start_world_trans = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation,
        };
        let mut body = RigidBody::new(info);
        body.user_idx = user_idx;
        body
    }

    fn make_world() -> DiscreteDynamicsWorld {
        let broadphase = GridBroadphase::new(
            Vec3A::new(-5600.0, -6000.0, 0.0) * UU_TO_BT,
            Vec3A::new(5600.0, 6000.0, 2200.0) * UU_TO_BT,
            370.0 * UU_TO_BT,
            8,
        );
        DiscreteDynamicsWorld::new(broadphase, Vec3A::ZERO)
    }

    fn contact_info(contact_point: Vec3A, ground_body_idx: usize) -> RaycastInfo {
        RaycastInfo {
            contact_normal: Vec3A::Z,
            contact_point,
            ground_body_idx,
            suspension_length: 0.5,
            impulse: Vec3A::ZERO,
            ground_stick: Vec3A::ZERO,
            is_in_contact_with_world: false,
            clipped_inv_contact_dot_suspension: 1.0,
            suspension_relative_vel: 0.0,
        }
    }

    #[test]
    fn car_victim_gets_opposite_reaction() {
        let mut world = make_world();
        let chassis_idx = world.add_rigid_body_default(make_body(
            180.0,
            Vec3A::new(0.0, 0.0, 5.0),
            UserInfoTypes::Car,
        ));
        let victim_idx =
            world.add_rigid_body_default(make_body(180.0, Vec3A::ZERO, UserInfoTypes::Car));
        let contact = Vec3A::new(0.5, 0.0, 1.0);
        let extra = 2.0;
        let full = extra * NUM_WHEELS as f32;

        let inv_mass = world.bodies()[victim_idx].inv_mass;
        let tensor = world.bodies()[victim_idx].inv_inertia_tensor_world;
        let offset = contact - world.bodies()[victim_idx].get_world_trans().translation;
        let expected_lin = -Vec3A::Z * full * inv_mass;
        let expected_ang = tensor * offset.cross(-Vec3A::Z * full);
        assert!(expected_ang.y > 0.0);

        let mut vehicle = VehicleRL::new(
            chassis_idx,
            [crate::bullet::dynamics::vehicle::WheelInfo::DEFAULT; NUM_WHEELS],
        );
        vehicle.wheels[0].extra_pushback = extra;
        vehicle.wheels[0].raycast_info = Some(contact_info(contact, victim_idx));
        vehicle.apply_hit_car_pushback(&mut world);

        let victim = &world.bodies()[victim_idx];
        assert!((victim.lin_vel - expected_lin).length() < 1e-5);
        assert!((victim.ang_vel - expected_ang).length() < 1e-5);
        assert_eq!(world.bodies()[chassis_idx].lin_vel, Vec3A::ZERO);
    }

    #[test]
    fn self_hit_applies_no_reaction() {
        let mut world = make_world();
        let chassis_idx =
            world.add_rigid_body_default(make_body(180.0, Vec3A::ZERO, UserInfoTypes::Car));
        let mut vehicle = VehicleRL::new(
            chassis_idx,
            [crate::bullet::dynamics::vehicle::WheelInfo::DEFAULT; NUM_WHEELS],
        );
        vehicle.wheels[0].extra_pushback = 2.0;
        vehicle.wheels[0].raycast_info = Some(contact_info(Vec3A::new(0.5, 0.0, 1.0), chassis_idx));
        vehicle.apply_hit_car_pushback(&mut world);
        assert_eq!(world.bodies()[chassis_idx].lin_vel, Vec3A::ZERO);
        assert_eq!(world.bodies()[chassis_idx].ang_vel, Vec3A::ZERO);
    }

    #[test]
    fn ball_victim_applies_no_reaction() {
        let mut world = make_world();
        let chassis_idx = world.add_rigid_body_default(make_body(
            180.0,
            Vec3A::new(0.0, 0.0, 5.0),
            UserInfoTypes::Car,
        ));
        let ball_idx =
            world.add_rigid_body_default(make_body(30.0, Vec3A::ZERO, UserInfoTypes::Ball));
        let mut vehicle = VehicleRL::new(
            chassis_idx,
            [crate::bullet::dynamics::vehicle::WheelInfo::DEFAULT; NUM_WHEELS],
        );
        vehicle.wheels[0].extra_pushback = 2.0;
        vehicle.wheels[0].raycast_info = Some(contact_info(Vec3A::new(0.5, 0.0, 1.0), ball_idx));
        vehicle.apply_hit_car_pushback(&mut world);
        assert_eq!(world.bodies()[ball_idx].lin_vel, Vec3A::ZERO);
        assert_eq!(world.bodies()[ball_idx].ang_vel, Vec3A::ZERO);
    }

    #[test]
    fn zero_pushback_applies_no_reaction() {
        let mut world = make_world();
        let chassis_idx = world.add_rigid_body_default(make_body(
            180.0,
            Vec3A::new(0.0, 0.0, 5.0),
            UserInfoTypes::Car,
        ));
        let victim_idx =
            world.add_rigid_body_default(make_body(180.0, Vec3A::ZERO, UserInfoTypes::Car));
        let mut vehicle = VehicleRL::new(
            chassis_idx,
            [crate::bullet::dynamics::vehicle::WheelInfo::DEFAULT; NUM_WHEELS],
        );
        vehicle.wheels[0].extra_pushback = 0.0;
        vehicle.wheels[0].raycast_info = Some(contact_info(Vec3A::new(0.5, 0.0, 1.0), victim_idx));
        vehicle.apply_hit_car_pushback(&mut world);
        assert_eq!(world.bodies()[victim_idx].lin_vel, Vec3A::ZERO);
        assert_eq!(world.bodies()[victim_idx].ang_vel, Vec3A::ZERO);
    }
}
