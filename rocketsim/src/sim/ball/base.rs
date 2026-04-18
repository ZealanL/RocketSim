use std::f32::consts::TAU;

use glam::{Affine3A, Vec2, Vec3A};

use crate::{
    BallState, Car, GameMode, MutatorConfig,
    bullet::{
        collision::{
            broadphase::CollisionFilterGroups,
            shapes::{
                collision_shape::CollisionShapes, convex_hull_shape::ConvexHullShape,
                sphere_shape::SphereShape,
            },
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{ActivationState, CollisionFlags, RigidBody, RigidBodyConstructionInfo},
        },
        linear_math::angle::Angle,
    },
    consts::{UU_TO_BT, dropshot, heatseeker, snowday},
    sim::{UserInfoTypes, collision_masks::CollisionMasks, consts},
};

pub(crate) struct Ball {
    pub state: BallState,
    pub rigid_body_idx: usize,
    pub ground_stick_applied: bool,
    pub vel_impulse_cache: Vec3A,
}

impl Ball {
    fn make_ball_collision_shape(
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) -> (CollisionShapes, Vec3A) {
        if game_mode == GameMode::Snowday {
            let ang_step = TAU / f32::from(snowday::PUCK_CIRCLE_POINT_AMOUNT);
            let mut cur_ang = 0f32;
            let points = (0..snowday::PUCK_CIRCLE_POINT_AMOUNT)
                .flat_map(|_| {
                    const NEG_Z: Vec3A = Vec3A::new(1.0, 1.0, -1.0);

                    let point = Vec3A::new(
                        cur_ang.cos() * mutator_config.ball_radius,
                        cur_ang.sin() * mutator_config.ball_radius,
                        snowday::PUCK_HEIGHT / 2.0,
                    ) * UU_TO_BT;

                    cur_ang += ang_step;

                    [point, point * NEG_Z]
                })
                .collect();

            let shape = ConvexHullShape::new(points);
            let local_inertia = shape.calculate_local_intertia(mutator_config.ball_mass);

            (CollisionShapes::ConvexHull(shape), local_inertia)
        } else {
            let shape = SphereShape::new(mutator_config.ball_radius * UU_TO_BT);
            let local_inertia = shape.calculate_local_inertia(mutator_config.ball_mass);

            (CollisionShapes::Sphere(shape), local_inertia)
        }
    }

    pub fn new(
        game_mode: GameMode,
        bullet_world: &mut DiscreteDynamicsWorld,
        mutator_config: &MutatorConfig,
        no_rot: bool,
    ) -> Self {
        let (collision_shape, local_inertia) =
            Self::make_ball_collision_shape(game_mode, mutator_config);

        let mut info =
            RigidBodyConstructionInfo::new(mutator_config.ball_mass, collision_shape, true);
        info.start_world_trans.translation.z = consts::ball::REST_Z * UU_TO_BT;
        info.local_inertia = local_inertia;
        info.linear_damping = mutator_config.ball_drag;

        let coefs = if game_mode == GameMode::Snowday {
            consts::snowday::PUCK_COEFS
        } else {
            consts::ball::COEFS
        };
        info.friction = coefs.friction;
        info.restitution = coefs.restitution;

        let mut body = RigidBody::new(info);
        body.user_idx = UserInfoTypes::Ball;
        body.collision_flags |= CollisionFlags::CustomMaterialCallback as u8;
        body.no_rot = no_rot && matches!(body.get_collision_shape(), CollisionShapes::Sphere(_));

        let rigid_body_idx = bullet_world.add_rigid_body(
            body,
            CollisionFilterGroups::Default as u8
                | CollisionMasks::HoopsNet as u8
                | CollisionMasks::DropshotTile as u8,
            CollisionFilterGroups::All as u8,
        );

        Self {
            state: BallState::DEFAULT,
            rigid_body_idx,
            ground_stick_applied: false,
            vel_impulse_cache: Vec3A::ZERO,
        }
    }

    pub fn set_state(&mut self, rb: &mut RigidBody, state: BallState) {
        debug_assert_eq!(rb.world_array_idx, self.rigid_body_idx);
        debug_assert_eq!(rb.user_idx, UserInfoTypes::Ball);

        rb.set_world_trans(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * UU_TO_BT,
        });

        rb.set_lin_vel(state.phys.vel * UU_TO_BT);
        rb.set_ang_vel(state.phys.ang_vel);
        rb.update_inertia_tensor();

        if state.phys.vel != Vec3A::ZERO || state.phys.ang_vel != Vec3A::ZERO {
            rb.set_activation_state(ActivationState::Active);
        }

        self.state = state;
    }

    pub(crate) fn pre_tick_update(&mut self, rb: &mut RigidBody, game_mode: GameMode) {
        match game_mode {
            GameMode::Heatseeker => {
                if self.state.hs_info.y_target_dir == 0 {
                    return;
                }

                let vel_angle = Angle::from(self.state.vel);

                // Determine angle to goal
                let goal_target_pos = Vec3A::new(
                    0.0,
                    heatseeker::TARGET_Y * f32::from(self.state.hs_info.y_target_dir),
                    heatseeker::TARGET_Z,
                );
                let angle_to_goal = Angle::from(goal_target_pos - self.state.phys.pos);

                // Find difference between target angle and current angle
                let delta_angle = angle_to_goal - vel_angle;

                // Determine speed ratio
                let cur_speed = self.state.phys.vel.length();
                let speed_ratio = cur_speed / heatseeker::MAX_SPEED;

                let mut new_angle = vel_angle;
                let base_interp_factor = speed_ratio * consts::TICK_TIME;
                new_angle.yaw +=
                    delta_angle.yaw * base_interp_factor * heatseeker::HORIZONTAL_BLEND;
                new_angle.pitch +=
                    delta_angle.pitch * base_interp_factor * heatseeker::VERTICAL_BLEND;
                new_angle.yaw = new_angle.yaw.rem_euclid(TAU);
                new_angle.pitch = new_angle
                    .pitch
                    .clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);
                new_angle.normalize_fix();

                // Limit pitch
                new_angle.pitch = new_angle
                    .pitch
                    .clamp(-heatseeker::MAX_TURN_PITCH, heatseeker::MAX_TURN_PITCH);

                // Apply aggressive UE3 rotator rounding
                // (This is suprisingly important for accuracy)
                new_angle = new_angle.round_ue3();

                // Determine new interpolated speed
                let new_speed = cur_speed
                    + (self.state.hs_info.cur_target_speed - cur_speed) * heatseeker::SPEED_BLEND;

                // Update velocity
                let new_dir = new_angle.get_forward_vec();
                let new_vel = new_dir * new_speed;
                rb.set_lin_vel(new_vel * UU_TO_BT);

                self.state.hs_info.time_since_hit += consts::TICK_TIME;
            }
            GameMode::Snowday => self.ground_stick_applied = false,
            GameMode::Dropshot | GameMode::Hoops => {
                // Launch ball after a short delay on kickoff
                let is_dropshot = game_mode == GameMode::Dropshot;

                let launch_delay = if is_dropshot {
                    dropshot::BALL_LAUNCH_DELAY
                } else {
                    consts::ball::HOOPS_LAUNCH_DELAY
                };

                let cur_kickoff_time =
                    self.state.tick_count_since_kickoff as f32 * consts::TICK_TIME;
                let prev_kickoff_time = cur_kickoff_time - consts::TICK_TIME;

                if prev_kickoff_time < launch_delay && cur_kickoff_time >= launch_delay {
                    // Launch triggered
                    // Make sure the ball is frozen at the kickoff X and Y
                    if self.state.phys.vel == Vec3A::ZERO
                        && self.state.phys.ang_vel == Vec3A::ZERO
                        && self.state.phys.pos.truncate() == Vec2::ZERO
                    {
                        // Apply the force
                        let launch_vel_z = if is_dropshot {
                            dropshot::BALL_LAUNCH_Z_VEL
                        } else {
                            consts::ball::HOOPS_LAUNCH_Z_VEL
                        };

                        rb.apply_central_impulse(
                            Vec3A::new(0.0, 0.0, launch_vel_z) * rb.get_mass() * UU_TO_BT,
                        );
                        rb.set_activation_state(ActivationState::Active);
                    }
                }
            }
            _ => {}
        }
    }

    pub(crate) fn finish_physics_tick(
        &mut self,
        rb: &mut RigidBody,
        mutator_config: &MutatorConfig,
    ) {
        if self.vel_impulse_cache.length_squared() != 0.0 {
            rb.lin_vel += self.vel_impulse_cache * UU_TO_BT;
            self.vel_impulse_cache = Vec3A::ZERO;
        }

        let ball_max_speed_bt = mutator_config.ball_max_speed * UU_TO_BT;
        if rb.lin_vel.length_squared() > ball_max_speed_bt * ball_max_speed_bt {
            rb.lin_vel = rb.lin_vel.normalize_or_zero() * ball_max_speed_bt;
        }

        if rb.ang_vel.length_squared() > consts::ball::MAX_ANG_SPEED * consts::ball::MAX_ANG_SPEED {
            rb.ang_vel = rb.ang_vel.normalize_or_zero() * consts::ball::MAX_ANG_SPEED;
        }

        self.state.phys.vel = rb.lin_vel * consts::BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;

        let trans = *rb.get_world_trans();
        self.state.phys.pos = trans.translation * consts::BT_TO_UU;
        self.state.phys.rot_mat = trans.matrix3;

        self.state.tick_count_since_kickoff += 1;
    }

    pub(crate) fn on_hit(
        &mut self,
        car: &mut Car,
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
        tick_count: u64,
    ) {
        let car_forward = car.state.phys.rot_mat.x_axis;
        let rel_pos = self.state.phys.pos - car.state.phys.pos;
        let rel_vel = self.state.phys.vel - car.state.phys.vel;

        let rel_speed = rel_vel
            .length()
            .min(consts::ball::car_hit_impulse::MAX_DELTA_VEL_UU);

        // Prevent repeated extra impulses
        let can_accel = if let Some(last_hit_tick) = self.state.last_extra_hit_tick {
            last_hit_tick < tick_count - 1
        } else {
            true
        };

        if rel_speed > 0.0 && can_accel {
            let extra_z_scale = game_mode == GameMode::Hoops
                && car.state.is_on_ground
                && car.state.phys.rot_mat.z_axis.z
                    > consts::ball::car_hit_impulse::Z_SCALE_HOOPS_NORMAL_Z_THRESH;
            let z_scale = if extra_z_scale {
                consts::ball::car_hit_impulse::Z_SCALE_HOOPS_GROUND
            } else {
                consts::ball::car_hit_impulse::Z_SCALE_NORMAL
            };

            let mut hit_dir = rel_pos * Vec3A::new(1.0, 1.0, z_scale).normalize_or_zero();
            let forward_dir_adjustment = car_forward
                * hit_dir.dot(car_forward)
                * const { 1.0 - consts::ball::car_hit_impulse::FORWARD_SCALE };
            hit_dir = (hit_dir - forward_dir_adjustment).normalize_or_zero();

            let added_vel = hit_dir
                * rel_speed
                * consts::curves::BALL_CAR_EXTRA_IMPULSE_FACTOR.get_output(rel_speed)
                * mutator_config.ball_hit_extra_force_scale;
            self.vel_impulse_cache += added_vel;
            self.state.last_extra_hit_tick = Some(tick_count);
        }

        match game_mode {
            GameMode::Heatseeker => {
                let can_increase = self.state.hs_info.time_since_hit
                    > heatseeker::MIN_SPEEDUP_INTERVAL
                    || self.state.hs_info.y_target_dir == 0;

                // Blue -> 1, Orange -> -1
                let new_target_dir = car.team as i8 * -2 + 1;
                if can_increase && new_target_dir != self.state.hs_info.y_target_dir {
                    self.state.hs_info.time_since_hit = 0.0;
                    self.state.hs_info.cur_target_speed = heatseeker::MAX_SPEED.min(
                        self.state.hs_info.cur_target_speed + heatseeker::TARGET_SPEED_INCREMENT,
                    );
                }

                self.state.hs_info.y_target_dir = new_target_dir;
            }
            GameMode::Dropshot => {
                let accumulated_hit_force = &mut self.state.ds_info.accumulated_hit_force;
                let charge_level = &mut self.state.ds_info.charge_level;

                let dir_from_car = (self.state.phys.pos - car.state.phys.pos).normalize_or_zero();
                let rel_vel_from_car = car.state.phys.vel - self.state.phys.vel;
                let vel_info_ball = dir_from_car.dot(rel_vel_from_car);

                if vel_info_ball >= dropshot::MIN_CHARGE_HIT_SPEED {
                    *accumulated_hit_force += vel_info_ball;

                    if *accumulated_hit_force >= dropshot::MIN_ABSORBED_FORCE_FOR_SUPERCHARGE {
                        *charge_level = 3;
                    } else if *accumulated_hit_force >= dropshot::MIN_ABSORBED_FORCE_FOR_CHARGE {
                        *charge_level = 2;
                    }
                }

                if *charge_level > 1 {
                    // Blue -> 1, Orange -> -1
                    self.state.ds_info.y_target_dir = car.team as i8 * -2 + 1;
                }
            }
            _ => {}
        };
    }

    pub fn on_world_hit(&mut self, rb: &mut RigidBody, game_mode: GameMode, normal: Vec3A) {
        match game_mode {
            GameMode::Heatseeker => {
                const ARENA_EXTENT: Vec3A = consts::arena::get_aabb(GameMode::Soccar).max;
                if self.state.hs_info.y_target_dir == 0 {
                    return;
                }

                let y_target_dir = f32::from(self.state.hs_info.y_target_dir);
                let rel_normal_y = normal.y * y_target_dir;
                let rel_y = self.state.phys.pos.y * y_target_dir;
                if rel_normal_y <= -heatseeker::WALL_BOUNCE_CHANGE_Y_NORMAL
                    && rel_y >= ARENA_EXTENT.y - heatseeker::WALL_BOUNCE_CHANGE_Y_THRESH
                {
                    // We hit far enough to change direction
                    self.state.hs_info.y_target_dir *= -1;

                    let goal_target_pos = Vec3A::new(
                        0.0,
                        heatseeker::TARGET_Y * y_target_dir,
                        heatseeker::TARGET_Z,
                    );

                    // Add wall bounce impulse
                    let dir_to_goal = (goal_target_pos - self.state.phys.pos).normalize_or_zero();

                    let bounce_dir = dir_to_goal * (1.0 - heatseeker::WALL_BOUNCE_UP_FRAC)
                        + Vec3A::Z * heatseeker::WALL_BOUNCE_UP_FRAC;
                    let bounce_impulse = bounce_dir
                        * self.state.phys.vel.length()
                        * heatseeker::WALL_BOUNCE_FORCE_SCALE;
                    self.vel_impulse_cache += bounce_impulse * UU_TO_BT;
                }
            }
            GameMode::Snowday if !self.ground_stick_applied => {
                rb.apply_central_force(-normal * snowday::PUCK_GROUND_STICK_FORCE);
                self.ground_stick_applied = true;
            }
            _ => {}
        }
    }
}
