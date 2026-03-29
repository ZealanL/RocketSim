use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};
use std::f32::consts::PI;
use std::ops::{Deref, DerefMut};

// Shorthand using aliases for constants
use crate::bullet::dynamics::rigid_body::{ActivationState, CollisionFlags};
use crate::consts::{
    BT_TO_UU, TICK_TIME, UU_TO_BT, bullet_vehicle as vehicle_consts, car as car_consts,
    car::drive as drive_consts, curves,
};
use crate::sim::car::car_info::CarInfo;
use crate::{
    CarBodyConfig, CarControls, CarState, CollisionMasks, GameMode, MutatorConfig, PhysState, Team,
    bullet::{
        collision::{
            broadphase::CollisionFilterGroups,
            shapes::{
                box_shape::BoxShape, collision_shape::CollisionShapes,
                compound_shape::CompoundShape,
            },
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
            vehicle::{raycaster::VehicleRaycaster, vehicle_rl::VehicleRL},
        },
        linear_math::Mat3AExt,
    },
    consts,
    sim::UserInfoTypes,
};

pub struct Car {
    pub(crate) info: CarInfo,
    pub(crate) bullet_vehicle: VehicleRL,
    pub(crate) rigid_body_idx: usize,
    pub(crate) vel_impulse_cache: Vec3A,
    pub(crate) state: CarState,
}

impl Deref for Car {
    type Target = CarInfo;
    fn deref(&self) -> &Self::Target {
        &self.info
    }
}
impl DerefMut for Car {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.info
    }
}

impl Car {
    pub(crate) fn new(
        idx: usize,
        team: Team,
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        config: CarBodyConfig,
    ) -> Self {
        let child_hitbox_shape = BoxShape::new(config.hitbox_size * UU_TO_BT * 0.5);
        let local_inertia = child_hitbox_shape.calculate_local_intertia(car_consts::MASS_BT);

        let hitbox_offset = Affine3A {
            matrix3: Mat3A::IDENTITY,
            translation: config.hitbox_pos_offset * UU_TO_BT,
        };
        let compound_shape = CompoundShape::new(child_hitbox_shape, hitbox_offset);

        let collision_shape = CollisionShapes::Compound(compound_shape);
        let mut rb_info =
            RigidBodyConstructionInfo::new(car_consts::MASS_BT, collision_shape, false);
        rb_info.friction = car_consts::BASE_COEFS.friction;
        rb_info.restitution = car_consts::BASE_COEFS.restitution;
        rb_info.start_world_trans = Affine3A::IDENTITY;
        rb_info.local_inertia = local_inertia;

        let mut body = RigidBody::new(rb_info);
        body.user_idx = UserInfoTypes::Car;
        body.collision_flags |= CollisionFlags::CustomMaterialCallback as u8;

        let rigid_body_idx = bullet_world.add_rigid_body(
            body,
            CollisionFilterGroups::Default as u8 | CollisionMasks::DropshotFloor as u8,
            CollisionFilterGroups::All as u8,
        );

        let rb = &bullet_world.bodies()[rigid_body_idx];
        let raycaster = const { VehicleRaycaster::new(CollisionMasks::DropshotFloor as u8) };
        let mut bullet_vehicle = VehicleRL::new(rigid_body_idx, raycaster);

        let wheel_direction_cs = Vec3A::new(0.0, 0.0, -1.0);
        let wheel_axle_cs = Vec3A::new(0.0, -1.0, 0.0);

        for i in 0..4 {
            let front = i < 2;
            let left = i % 2 != 0;

            let wheels = if front {
                &config.front_wheels
            } else {
                &config.back_wheels
            };

            let radius = wheels.wheel_radius;
            let mut wheel_ray_start_offset = wheels.connection_point_offset;
            let suspension_rest_length =
                wheels.suspension_rest_length - vehicle_consts::MAX_SUSPENSION_TRAVEL;

            if left {
                wheel_ray_start_offset.y *= -1.0;
            }

            bullet_vehicle.add_wheel(
                rb,
                wheel_ray_start_offset * UU_TO_BT,
                wheel_direction_cs,
                wheel_axle_cs,
                suspension_rest_length * UU_TO_BT,
                radius * UU_TO_BT,
            );

            bullet_vehicle.wheels[i].suspsension_force_scale = if front {
                vehicle_consts::SUSPENSION_FORCE_SCALE_FRONT
            } else {
                vehicle_consts::SUSPENSION_FORCE_SCALE_BACK
            };
        }

        Self {
            info: CarInfo { idx, team, config },
            rigid_body_idx,
            bullet_vehicle,
            vel_impulse_cache: Vec3A::ZERO,
            state: CarState {
                boost: mutator_config.car_spawn_boost_amount,
                ..Default::default()
            },
        }
    }

    /// - `respawn_delay` by default is `rocketsim::consts::DEMO_RESPAWN_TIME`
    pub(crate) const fn demolish(&mut self, respawn_delay: f32) {
        self.state.is_demoed = true;
        self.state.demo_respawn_timer = respawn_delay;
    }

    /// - `boost_amount` by default is `rocketsim::consts::BOOST_RESPAWN_AMOUNT`
    ///
    /// Respawn the car, called after we have been demolished and waited for the respawn timer
    pub(crate) fn respawn(
        &mut self,
        rb: &mut RigidBody,
        rng: &mut Rng,
        game_mode: GameMode,
        boost_amount: f32,
    ) {
        let respawn_locations = car_consts::spawn::get_respawn_locations(game_mode);
        let spawn_pos_idx = rng.usize(0..respawn_locations.len());
        let spawn_pos = respawn_locations[spawn_pos_idx];

        let new_state = CarState {
            phys: PhysState {
                pos: Vec3A::new(
                    spawn_pos.x,
                    spawn_pos.y * -self.team.get_y_dir(),
                    car_consts::spawn::RESPAWN_Z,
                ),
                rot_mat: Mat3A::from_euler(
                    EulerRot::ZYX,
                    spawn_pos.yaw_ang + if self.team == Team::Blue { 0.0 } else { PI },
                    0.0,
                    0.0,
                ),
                vel: Vec3A::ZERO,
                ang_vel: Vec3A::ZERO,
            },
            boost: boost_amount,
            ..Default::default()
        };

        self.set_state(rb, &new_state);
    }

    pub const fn get_state(&self) -> &CarState {
        &self.state
    }

    pub const fn get_config(&self) -> &CarBodyConfig {
        &self.info.config
    }

    pub const fn set_controls(&mut self, new_controls: CarControls) {
        self.state.controls = new_controls;
    }

    pub(crate) fn set_state(&mut self, rb: &mut RigidBody, state: &CarState) {
        debug_assert_eq!(rb.user_idx, UserInfoTypes::Car);
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);

        rb.set_world_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });

        rb.lin_vel = state.phys.vel * UU_TO_BT;
        rb.ang_vel = state.phys.ang_vel;
        rb.update_inertia_tensor();

        self.vel_impulse_cache = Vec3A::ZERO;
        self.state = *state;
    }

    fn update_wheels(
        &mut self,
        rb: &mut RigidBody,
        num_wheels_in_contact: u8,
        forward_speed_uu: f32,
    ) {
        let handbrake_delta = if self.state.controls.handbrake {
            drive_consts::POWERSLIDE_RISE_RATE
        } else {
            -drive_consts::POWERSLIDE_FALL_RATE
        } * TICK_TIME;
        self.state.handbrake_val = (self.state.handbrake_val + handbrake_delta).clamp(0.0, 1.0);

        let mut real_brake = 0.0;
        let real_throttle = if self.state.controls.boost && self.state.boost > 0.0 {
            1.0
        } else {
            self.state.controls.throttle
        };

        let abs_forward_speed_uu = forward_speed_uu.abs();
        let mut engine_throttle = real_throttle;
        if !self.state.controls.handbrake {
            if real_throttle.abs() >= drive_consts::THROTTLE_DEADZONE {
                if abs_forward_speed_uu > drive_consts::STOPPING_FORWARD_VEL
                    && real_throttle.signum() != forward_speed_uu.signum()
                {
                    real_brake = 1.0;

                    if abs_forward_speed_uu > drive_consts::BRAKING_NO_THROTTLE_SPEED_THRESH {
                        engine_throttle = 0.0;
                    }
                }
            } else {
                engine_throttle = 0.0;
                real_brake = if abs_forward_speed_uu < drive_consts::STOPPING_FORWARD_VEL {
                    1.0
                } else {
                    drive_consts::COASTING_BRAKE_FACTOR
                };
            }
        }

        let mut drive_speed_scale =
            curves::DRIVE_SPEED_TORQUE_FACTOR.get_output(abs_forward_speed_uu);
        if num_wheels_in_contact < 3 {
            drive_speed_scale /= 4.0;
        }

        let drive_engine_force = engine_throttle
            * const { drive_consts::THROTTLE_TORQUE_AMOUNT * UU_TO_BT }
            * drive_speed_scale;
        let drive_brake_force = real_brake * const { drive_consts::BRAKE_TORQUE_AMOUNT * UU_TO_BT };
        for wheel in &mut self.bullet_vehicle.wheels {
            wheel.wheel_info.engine_force = drive_engine_force;
            wheel.wheel_info.brake = drive_brake_force;
        }

        let mut steer_angle = if self.config.three_wheels {
            curves::STEER_ANGLE_FROM_SPEED_THREEWHEEL.get_output(abs_forward_speed_uu)
        } else {
            curves::STEER_ANGLE_FROM_SPEED.get_output(abs_forward_speed_uu)
        };
        if self.state.handbrake_val != 0.0 {
            steer_angle += (curves::POWERSLIDE_STEER_ANGLE_FROM_SPEED
                .get_output(abs_forward_speed_uu)
                - steer_angle)
                * self.state.handbrake_val;
        }

        steer_angle *= self.state.controls.steer;
        self.bullet_vehicle.wheels[0].steer_angle = steer_angle;
        self.bullet_vehicle.wheels[1].steer_angle = steer_angle;

        let car_pos = rb.get_world_pos();
        let car_vel = rb.lin_vel;
        let car_ang_vel = rb.ang_vel;

        for wheel in &mut self.bullet_vehicle.wheels {
            if wheel.wheel_info.raycast_info.ground_obj.is_none() {
                continue;
            }

            let lat_dir = wheel.wheel_info.world_trans.matrix3.y_axis;
            let long_dir = lat_dir.cross(wheel.wheel_info.raycast_info.contact_normal_ws);

            let wheel_delta = wheel.wheel_info.raycast_info.hard_point_ws - car_pos;
            let cross_vec = (car_ang_vel.cross(wheel_delta) + car_vel) * BT_TO_UU;

            let base_friction = cross_vec.dot(lat_dir).abs();
            let friction_curve_input = if base_friction > 5.0 {
                base_friction / (cross_vec.dot(long_dir).abs() + base_friction)
            } else {
                0.0
            };

            let mut lat_friction = if self.info.config.three_wheels {
                curves::LAT_FRICTION_THREEWHEEL
            } else {
                curves::LAT_FRICTION
            }
            .get_output(friction_curve_input);
            let mut long_friction = 1.0;

            if self.state.handbrake_val != 0.0 {
                let handbrake_amount = self.state.handbrake_val;
                lat_friction *= 1.0 - curves::HANDBRAKE_LAT_FRICTION_FACTOR * handbrake_amount;
                long_friction *= 1.0
                    + (curves::HANDBRAKE_LONG_FRICTION_FACTOR.get_output(friction_curve_input)
                        - 1.0)
                        * handbrake_amount;
            }

            if real_throttle == 0.0 {
                // contact is not sticky
                let non_sticky_scale = curves::NON_STICKY_FRICTION_FACTOR
                    .get_output(wheel.wheel_info.raycast_info.contact_normal_ws.z);
                lat_friction *= non_sticky_scale;
                long_friction *= non_sticky_scale;
            }

            wheel.lat_friction = lat_friction;
            wheel.long_friction = long_friction;
        }

        let wheels_have_world_contact = self
            .bullet_vehicle
            .wheels
            .iter()
            .any(|wheel| wheel.is_in_contact_with_world);
        if wheels_have_world_contact {
            let upwards_dir = self.bullet_vehicle.get_upwards_dir_from_wheel_contacts(rb);

            let full_stick = real_throttle != 0.0
                || abs_forward_speed_uu > car_consts::drive::STOPPING_FORWARD_VEL;
            let mut sticky_force_scale = f32::from(!self.config.three_wheels) * 0.5;
            if full_stick {
                sticky_force_scale += 1.0 - upwards_dir.z.abs();
            }

            rb.apply_central_force(
                upwards_dir
                    * sticky_force_scale
                    * const { consts::GRAVITY_Z * UU_TO_BT * car_consts::MASS_BT },
            );
        }
    }

    fn update_air_torque(&mut self, rb: &mut RigidBody, update_air_control: bool) {
        let forward_dir = self.state.get_forward_dir();
        let right_dir = self.state.get_right_dir();
        let up_dir = self.state.get_up_dir();

        let dir_pitch = -right_dir;
        let dir_yaw = up_dir;
        let dir_roll = -forward_dir;

        if self.state.is_flipping {
            self.state.is_flipping =
                self.state.has_flipped && self.state.flip_time < car_consts::flip::TORQUE_TIME;
        }

        let mut do_air_control = false;
        if self.state.is_flipping {
            if self.state.flip_rel_torque == Vec3A::ZERO {
                do_air_control = true;
            } else {
                let mut rel_dodge_torque = self.state.flip_rel_torque;

                let mut pitch_scale = 1.0;
                if rel_dodge_torque.y != 0.0
                    && self.state.controls.pitch != 0.0
                    && rel_dodge_torque.y.signum() == self.state.controls.pitch.signum()
                {
                    pitch_scale = 1.0 - self.state.controls.pitch.abs().min(1.0);
                    do_air_control = true;
                }

                rel_dodge_torque.y *= pitch_scale;
                let dodge_torque = rel_dodge_torque
                    * const { Vec3A::new(car_consts::flip::TORQUE_X, car_consts::flip::TORQUE_Y, 0.0) };

                let rb_torque = rb.inv_inertia_tensor_world.bullet_inverse()
                    * rb.get_world_trans().matrix3
                    * dodge_torque;
                rb.apply_torque(rb_torque);
            }
        } else {
            do_air_control = true;
        }

        do_air_control &= !self.state.is_auto_flipping;
        do_air_control &= update_air_control;
        if do_air_control {
            let mut pitch_torque_scale = 1.0;
            let torque = if self.state.controls.pitch != 0.0
                || self.state.controls.yaw != 0.0
                || self.state.controls.roll != 0.0
            {
                if self.state.is_flipping
                    || self.state.has_flipped
                        && self.state.flip_time
                            < const {
                                car_consts::flip::TORQUE_TIME
                                    + car_consts::flip::PITCHLOCK_EXTRA_TIME
                            }
                {
                    pitch_torque_scale = 0.0;
                }

                self.state.controls.pitch
                    * dir_pitch
                    * pitch_torque_scale
                    * car_consts::air_control::TORQUE.x
                    + self.state.controls.yaw * dir_yaw * car_consts::air_control::TORQUE.y
                    + self.state.controls.roll * dir_roll * car_consts::air_control::TORQUE.z
            } else {
                Vec3A::ZERO
            };

            let ang_vel = rb.ang_vel;

            let damp_pitch = dir_pitch.dot(ang_vel)
                * car_consts::air_control::DAMPING.x
                * (1.0 - (self.state.controls.pitch * pitch_torque_scale).abs());
            let damp_yaw = dir_yaw.dot(ang_vel)
                * car_consts::air_control::DAMPING.y
                * (1.0 - self.state.controls.yaw.abs());
            let damp_roll = dir_roll.dot(ang_vel) * car_consts::air_control::DAMPING.z;

            let damping = dir_yaw * damp_yaw + dir_pitch * damp_pitch + dir_roll * damp_roll;

            let rb_torque = rb.inv_inertia_tensor_world.bullet_inverse()
                * (torque - damping)
                * car_consts::air_control::TORQUE_APPLY_SCALE;
            rb.apply_torque(rb_torque);
        }

        if self.state.controls.throttle != 0.0 {
            rb.apply_central_force(
                forward_dir
                    * self.state.controls.throttle
                    * const { car_consts::drive::THROTTLE_AIR_ACCEL * UU_TO_BT * car_consts::MASS_BT },
            );
        }
    }

    fn update_jump(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
    ) {
        let up_dir = self.state.get_up_dir();

        if self.state.is_on_ground && self.state.is_jumping {
            if self.state.has_jumped
                && self.state.jump_time
                    < const { car_consts::jump::MIN_TIME + car_consts::jump::RESET_TIME_PAD }
            {
                // Don't reset the jump just yet, we might still be leaving the ground
                // This fixes the bug where jump is reset before we actually leave the ground after a minimum-time jump
                // TODO: RL does something similar to this time-pad, but not exactly the same
            } else {
                self.state.has_jumped = false;
                self.state.jump_time = 0.0;
            }
        }

        if self.state.is_jumping {
            self.state.is_jumping = self.state.jump_time < car_consts::jump::MIN_TIME
                || (self.state.controls.jump && self.state.jump_time < car_consts::jump::MAX_TIME);
        } else if self.state.is_on_ground && jump_pressed {
            self.state.is_jumping = true;
            self.state.jump_time = 0.0;
            let jump_start_force = up_dir
                * mutator_config.jump_immediate_force
                * const { UU_TO_BT * car_consts::MASS_BT };
            rb.apply_central_impulse(jump_start_force);
        }

        if self.state.is_jumping {
            self.state.has_jumped = true;

            let mut total_jump_force = up_dir * mutator_config.jump_accel;
            if self.state.jump_time < car_consts::jump::MIN_TIME {
                // TODO: Temporary fix for unknown accuracy issue (thus not in constants)
                const JUMP_PRE_MIN_ACCEL_SCALE: f32 = 0.62;
                total_jump_force *= JUMP_PRE_MIN_ACCEL_SCALE;
            }

            rb.apply_central_force(total_jump_force * const { UU_TO_BT * car_consts::MASS_BT });
        }

        if self.state.is_jumping || self.state.has_jumped {
            self.state.jump_time += TICK_TIME;
        }
    }

    fn update_auto_flip(&mut self, rb: &mut RigidBody, jump_pressed: bool) {
        if jump_pressed
            && self
                .state
                .world_contact_normal
                .is_some_and(|world_contact_normal| {
                    world_contact_normal.z > car_consts::autoflip::NORM_Z_THRESH
                })
        {
            let (_, _, roll) = self.state.phys.rot_mat.to_euler(EulerRot::ZYX);
            let abs_roll = roll.abs();
            if abs_roll > car_consts::autoflip::ROLL_THRESH {
                self.state.auto_flip_timer = car_consts::autoflip::TIME * (abs_roll / PI);
                self.state.auto_flip_torque_scale = roll.signum();
                self.state.is_auto_flipping = true;

                rb.apply_central_impulse(
                    -self.state.get_up_dir()
                        * const { car_consts::autoflip::IMPULSE * UU_TO_BT * car_consts::MASS_BT },
                );
            }
        }

        if self.state.is_auto_flipping {
            if self.state.auto_flip_timer <= 0.0 {
                self.state.is_auto_flipping = false;
                self.state.auto_flip_timer = 0.0;
            } else {
                rb.ang_vel += self.state.get_forward_dir()
                    * car_consts::autoflip::TORQUE
                    * self.state.auto_flip_torque_scale
                    * TICK_TIME;
                self.state.auto_flip_timer -= TICK_TIME;
            }
        }
    }

    fn update_double_jump_or_flip(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
        jump_pressed: bool,
        forward_speed_uu: f32,
    ) {
        if self.state.is_on_ground {
            self.state.has_double_jumped = false;
            self.state.has_flipped = false;
            self.state.air_time = 0.0;
            self.state.air_time_since_jump = 0.0;
            self.state.flip_time = 0.0;
            return;
        }

        self.state.air_time += TICK_TIME;

        if self.state.has_jumped && !self.state.is_jumping {
            self.state.air_time_since_jump += TICK_TIME;
        } else {
            self.state.air_time_since_jump = 0.0;
        }

        if jump_pressed && self.state.air_time_since_jump < car_consts::jump::DOUBLEJUMP_MAX_DELAY {
            let input_magnitude = self.state.controls.yaw.abs()
                + self.state.controls.pitch.abs()
                + self.state.controls.roll.abs();
            let is_flip_input = input_magnitude >= self.config.dodge_deadzone;

            let can_use = !self.state.is_auto_flipping
                && !self.state.has_double_jumped
                && !self.state.has_flipped
                || if is_flip_input {
                    mutator_config.unlimited_flips
                } else {
                    mutator_config.unlimited_double_jumps
                };

            if can_use {
                if is_flip_input {
                    self.state.flip_time = 0.0;
                    self.state.has_flipped = true;
                    self.state.is_flipping = true;

                    let forward_speed_ratio = forward_speed_uu.abs() / car_consts::MAX_SPEED;
                    let mut dodge_dir = Vec3A::new(
                        -self.state.controls.pitch,
                        self.state.controls.yaw + self.state.controls.roll,
                        0.0,
                    );

                    if dodge_dir.x.abs() < 0.1 && dodge_dir.y.abs() < 0.1 {
                        dodge_dir = Vec3A::ZERO;
                    } else {
                        dodge_dir = dodge_dir.normalize_or_zero();
                    }

                    self.state.flip_rel_torque = Vec3A::new(-dodge_dir.y, dodge_dir.x, 0.0);

                    if dodge_dir.x.abs() < 0.1 {
                        dodge_dir.x = 0.0;
                    }

                    if dodge_dir.y.abs() < 0.1 {
                        dodge_dir.y = 0.0;
                    }

                    if dodge_dir.length_squared() > const { f32::EPSILON * f32::EPSILON } {
                        let should_dodge_backwards = if forward_speed_uu.abs() < 100. {
                            dodge_dir.x.is_sign_negative()
                        } else {
                            dodge_dir.x.signum() != forward_speed_uu.signum()
                        };

                        let max_speed_scale_x = if should_dodge_backwards {
                            car_consts::flip::BACKWARD_IMPULSE_MAX_SPEED_SCALE
                        } else {
                            car_consts::flip::FORWARD_IMPULSE_MAX_SPEED_SCALE
                        };

                        let mut initial_dodge_vel = dodge_dir * car_consts::flip::INITIAL_VEL_SCALE;
                        initial_dodge_vel.x *=
                            ((max_speed_scale_x - 1.) * forward_speed_ratio) + 1.0;
                        initial_dodge_vel.y *= ((car_consts::flip::SIDE_IMPULSE_MAX_SPEED_SCALE
                            - 1.)
                            * forward_speed_ratio)
                            + 1.0;
                        if should_dodge_backwards {
                            initial_dodge_vel.x *= car_consts::flip::BACKWARD_IMPULSE_SCALE_X;
                        }

                        let forward_dir_2d =
                            self.state.get_forward_dir().with_z(0.0).normalize_or_zero();
                        let right_dir_2d = Vec3A::new(-forward_dir_2d.y, forward_dir_2d.x, 0.0);
                        let final_delta_vel = initial_dodge_vel.x * forward_dir_2d
                            + initial_dodge_vel.y * right_dir_2d;

                        rb.apply_central_impulse(
                            final_delta_vel * const { UU_TO_BT * car_consts::MASS_BT },
                        );
                    }
                } else {
                    let jump_start_force = self.state.get_up_dir()
                        * const { car_consts::jump::IMMEDIATE_FORCE * UU_TO_BT * car_consts::MASS_BT };
                    rb.apply_central_impulse(jump_start_force);
                    self.state.has_double_jumped = true;
                }
            }
        }

        if self.state.is_flipping {
            self.state.flip_time += TICK_TIME;
            if self.state.flip_time <= car_consts::flip::TORQUE_TIME
                && self.state.flip_time >= car_consts::flip::Z_DAMP_START
                && (rb.lin_vel.z < 0.0 || self.state.flip_time < car_consts::flip::Z_DAMP_END)
            {
                rb.lin_vel.z *= 1.0 - car_consts::flip::Z_DAMP_120;
            }
        } else if self.state.has_flipped {
            self.state.flip_time += TICK_TIME;
        }
    }

    fn update_auto_roll(&self, rb: &mut RigidBody, num_wheels_in_contact: u8) {
        let ground_up_dir = if num_wheels_in_contact > 0 {
            self.bullet_vehicle.get_upwards_dir_from_wheel_contacts(rb)
        } else {
            self.state.world_contact_normal.unwrap()
        };

        let ground_down_dir = -ground_up_dir;

        let forward_dir = self.state.get_forward_dir();
        let right_dir = self.state.get_right_dir();

        let cross_right_dir = ground_up_dir.cross(forward_dir);
        let cross_forward_dir = ground_down_dir.cross(cross_right_dir);

        let right_torque_factor = 1.0 - right_dir.dot(cross_right_dir).clamp(0.0, 1.0);
        let forward_torque_factor = 1.0 - forward_dir.dot(cross_forward_dir).clamp(0.0, 1.0);

        let torque_dir_right = forward_dir * -right_dir.dot(ground_up_dir).signum();
        let torque_dir_forward = right_dir * forward_dir.dot(ground_up_dir).signum();

        let torque_right = torque_dir_right * right_torque_factor;
        let torque_forward = torque_dir_forward * forward_torque_factor;

        rb.apply_central_force(
            ground_down_dir
                * const { car_consts::autoroll::FORCE * UU_TO_BT * car_consts::MASS_BT },
        );

        let rb_torque = rb.inv_inertia_tensor_world.bullet_inverse()
            * (torque_forward + torque_right)
            * car_consts::autoroll::TORQUE;
        rb.apply_torque(rb_torque);
    }

    fn update_boost(&mut self, rb: &mut RigidBody, mutator_config: &MutatorConfig) {
        self.state.is_boosting = if self.state.boost > 0.0 {
            self.state.controls.boost
                || (self.state.is_boosting
                    && self.state.boosting_time < car_consts::boost::MIN_TIME)
        } else {
            false
        };

        if self.state.is_boosting {
            self.state.boosting_time += TICK_TIME;
            self.state.time_since_boosted = 0.0;
            self.state.boost -= mutator_config.boost_used_per_second * TICK_TIME;

            let accel = if self.state.is_on_ground {
                mutator_config.boost_accel_ground
            } else {
                mutator_config.boost_accel_air
            };

            rb.apply_central_force(
                accel * self.state.get_forward_dir() * (UU_TO_BT * car_consts::MASS_BT),
            );
        } else {
            self.state.boosting_time = 0.0;
            self.state.time_since_boosted += TICK_TIME;

            if mutator_config.recharge_boost_enabled
                && self.state.time_since_boosted >= mutator_config.recharge_boost_delay
            {
                self.state.boost += mutator_config.recharge_boost_per_second * TICK_TIME;
            }
        }

        self.state.boost = self.state.boost.clamp(0.0, car_consts::boost::MAX);
    }

    pub(crate) fn pre_tick_update(
        &mut self,
        collision_world: &mut DiscreteDynamicsWorld,
        rng: &mut Rng,
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) {
        debug_assert!(
            self.bullet_vehicle.get_num_wheels() == 4 || self.bullet_vehicle.get_num_wheels() == 3
        );

        let forward_speed_uu = {
            let rb = &mut collision_world.bodies_mut()[self.rigid_body_idx];
            if self.state.is_demoed {
                self.state.demo_respawn_timer = (self.state.demo_respawn_timer - TICK_TIME).max(0.0);
                if self.state.demo_respawn_timer == 0.0 {
                    self.respawn(rb, rng, game_mode, mutator_config.car_spawn_boost_amount);
                }

                rb.set_activation_state(ActivationState::DisableSimulation);
                rb.collision_flags |= CollisionFlags::NoContactResponse as u8;
            } else {
                rb.force_activate();
                rb.collision_flags &= !(CollisionFlags::NoContactResponse as u8);
            }

            if self.state.is_demoed {
                return;
            }

            self.state.controls = self.state.controls.clamp();

            rb.get_forward_speed() * BT_TO_UU
        };

        // Do first part of the btVehicleRL update (update wheel transforms, do traces, calculate friction impulses)
        self.bullet_vehicle
            .update_vehicle_first(collision_world, TICK_TIME);

        let jump_pressed = self.state.controls.jump && !self.state.prev_controls.jump;

        let mut num_wheels_in_contact = 0u8;
        for (wheel, has_contact) in self
            .bullet_vehicle
            .wheels
            .iter()
            .zip(&mut self.state.wheels_with_contact)
        {
            let in_contact = wheel.wheel_info.raycast_info.is_in_contact;
            *has_contact = in_contact;
            num_wheels_in_contact += u8::from(in_contact);
        }

        self.state.is_on_ground = num_wheels_in_contact >= 3;

        let rb = &mut collision_world.bodies_mut()[self.rigid_body_idx];

        self.update_wheels(rb, num_wheels_in_contact, forward_speed_uu);

        if self.state.is_on_ground {
            self.state.is_flipping = false;
        } else {
            self.update_air_torque(rb, num_wheels_in_contact == 0);
        }

        self.update_jump(rb, mutator_config, jump_pressed);
        self.update_auto_flip(rb, jump_pressed);
        self.update_double_jump_or_flip(rb, mutator_config, jump_pressed, forward_speed_uu);

        if self.state.controls.throttle != 0.0
            && ((0 < num_wheels_in_contact && num_wheels_in_contact < 4)
                || self.state.world_contact_normal.is_some())
        {
            self.update_auto_roll(rb, num_wheels_in_contact);
        }

        self.state.world_contact_normal = None;

        self.bullet_vehicle.update_vehicle_second(rb, TICK_TIME);
        self.update_boost(rb, mutator_config);
    }

    pub(crate) fn post_tick_update(&mut self, rb: &RigidBody) {
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);
        if self.state.is_demoed {
            return;
        }

        self.state.phys.rot_mat = rb.get_world_trans().matrix3;

        let speed_squared = (rb.lin_vel * BT_TO_UU).length_squared();
        self.state.is_supersonic = speed_squared
            >= if self.state.is_supersonic
                && self.state.supersonic_time < car_consts::supersonic::MAINTAIN_MAX_TIME
            {
                const {
                    car_consts::supersonic::MAINTAIN_MIN_SPEED
                        * car_consts::supersonic::MAINTAIN_MIN_SPEED
                }
            } else {
                const { car_consts::supersonic::START_SPEED * car_consts::supersonic::START_SPEED }
            };

        if self.state.is_supersonic {
            self.state.supersonic_time += TICK_TIME;
        } else {
            self.state.supersonic_time = 0.0;
        }

        self.state.bump_cooldown_timer = (self.state.bump_cooldown_timer - TICK_TIME).max(0.0);
        self.state.prev_controls = self.state.controls;
    }

    pub(crate) fn finish_physics_tick(&mut self, rb: &mut RigidBody) {
        const MAX_SPEED: f32 = car_consts::MAX_SPEED * UU_TO_BT;
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);

        if self.state.is_demoed {
            return;
        }

        if self.vel_impulse_cache != Vec3A::ZERO {
            rb.lin_vel += self.vel_impulse_cache * UU_TO_BT;
            self.vel_impulse_cache = Vec3A::ZERO;
        }

        let vel = &mut rb.lin_vel;
        if vel.length_squared() > const { MAX_SPEED * MAX_SPEED } {
            *vel = vel.normalize_or_zero() * MAX_SPEED;
        }

        let ang_vel = &mut rb.ang_vel;
        if ang_vel.length_squared()
            > const { car_consts::MAX_ANG_SPEED * car_consts::MAX_ANG_SPEED }
        {
            *ang_vel = ang_vel.normalize_or_zero() * car_consts::MAX_ANG_SPEED;
        }

        self.state.phys.pos = rb.get_world_trans().translation * BT_TO_UU;
        self.state.phys.vel = rb.lin_vel * BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;
    }
}
