use crate::bullet::collision::dispatch::collision_object::ActivationState;
use crate::consts::{UU_TO_BT, dropshot, heatseeker};
use crate::sim::UserInfoTypes;
use crate::{Arena, BallHitInfo, BallState, GameMode, Team, consts};
use glam::{Affine3A, Vec3A};

impl Arena {
    pub fn get_ball_state(&self) -> &BallState {
        &self.ball.state
    }

    pub fn set_ball_state(&mut self, state: BallState) {
        let ball = &mut self.ball;
        let rb = &mut self.bullet_world.bodies_mut()[ball.rigid_body_idx];

        debug_assert_eq!(rb.collision_object.world_array_index, ball.rigid_body_idx);
        debug_assert_eq!(rb.collision_object.user_index, UserInfoTypes::Ball);

        rb.collision_object.set_world_transform(Affine3A {
            matrix3: state.phys.rot_mat,
            translation: state.phys.pos * consts::UU_TO_BT,
        });

        rb.set_linear_velocity(state.phys.vel * consts::UU_TO_BT);
        rb.set_angular_velocity(state.phys.ang_vel);
        rb.update_inertia_tensor();

        if state.phys.vel != Vec3A::ZERO || state.phys.ang_vel != Vec3A::ZERO {
            rb.collision_object
                .set_activation_state(ActivationState::Active);
        }

        ball.state = state;
    }

    pub(crate) fn ball_pre_tick_update(&mut self) {
        match self.game_mode {
            GameMode::Heatseeker => todo!(),
            GameMode::Snowday => self.ball.ground_stick_applied = false,
            GameMode::Dropshot | GameMode::Hoops => {
                // launch ball after a short delay on kickoff
                todo!()
            }
            _ => {}
        }
    }

    pub(crate) fn ball_finish_physics_tick(&mut self) {
        let ball = &mut self.ball;

        let rb = &mut self.bullet_world.bodies_mut()[ball.rigid_body_idx];

        if ball.velocity_impulse_cache.length_squared() != 0.0 {
            rb.linear_velocity += ball.velocity_impulse_cache * UU_TO_BT;
            ball.velocity_impulse_cache = Vec3A::ZERO;
        }

        let ball_max_speed_bt = self.mutator_config.ball_max_speed * consts::UU_TO_BT;
        if rb.linear_velocity.length_squared() > ball_max_speed_bt * ball_max_speed_bt {
            rb.linear_velocity = rb.linear_velocity.normalize() * ball_max_speed_bt;
        }

        if rb.angular_velocity.length_squared()
            > consts::ball::MAX_ANG_SPEED * consts::ball::MAX_ANG_SPEED
        {
            rb.angular_velocity = rb.angular_velocity.normalize() * consts::ball::MAX_ANG_SPEED;
        }

        ball.state.phys.vel = rb.linear_velocity * consts::BT_TO_UU;
        ball.state.phys.ang_vel = rb.angular_velocity;

        let trans = *rb.collision_object.get_world_transform();
        ball.state.phys.pos = trans.translation * consts::BT_TO_UU;
        ball.state.phys.rot_mat = trans.matrix3;
    }
}

impl Arena {
    pub(crate) fn on_ball_hit(&mut self, car_idx: usize, rel_pos: Vec3A) {
        let tick_count = self.tick_count;
        let ball_state = &mut self.ball.state;
        let car = self.cars.get_mut(car_idx).unwrap();

        let mut ball_hit_info = BallHitInfo {
            relative_pos_on_ball: rel_pos,
            tick_count_when_hit: tick_count,
            ball_pos: ball_state.phys.pos,
            extra_hit_vel: Vec3A::ZERO,
            tick_count_when_extra_impulse_applied: 0,
        };

        if let Some(old_bhi) = car.state.ball_hit_info {
            ball_hit_info.tick_count_when_extra_impulse_applied =
                old_bhi.tick_count_when_extra_impulse_applied;

            // Once we do an extra car-ball impulse, we need to wait at least 1 tick to do it again
            if tick_count <= old_bhi.tick_count_when_extra_impulse_applied + 1
                && old_bhi.tick_count_when_extra_impulse_applied <= tick_count
            {
                car.state.ball_hit_info = Some(ball_hit_info);
                return;
            }
        }

        ball_hit_info.tick_count_when_extra_impulse_applied = tick_count;

        let car_forward = car.state.phys.rot_mat.x_axis;
        let rel_pos = ball_state.phys.pos - car.state.phys.pos;
        let rel_vel = ball_state.phys.vel - car.state.phys.vel;

        let rel_speed = rel_vel
            .length()
            .min(consts::ball::car_hit_impulse::MAX_DELTA_VEL_UU);
        if rel_speed > 0.0 {
            let extra_z_scale = self.game_mode == GameMode::Hoops
                && car.state.is_on_ground
                && car.state.phys.rot_mat.z_axis.z
                    > consts::ball::car_hit_impulse::Z_SCALE_HOOPS_NORMAL_Z_THRESH;
            let z_scale = if extra_z_scale {
                consts::ball::car_hit_impulse::Z_SCALE_HOOPS_GROUND
            } else {
                consts::ball::car_hit_impulse::Z_SCALE_NORMAL
            };

            let mut hit_dir = rel_pos * Vec3A::new(1.0, 1.0, z_scale).normalize();
            let forward_dir_adjustment = car_forward
                * hit_dir.dot(car_forward)
                * const { 1.0 - consts::ball::car_hit_impulse::FORWARD_SCALE };
            hit_dir = (hit_dir - forward_dir_adjustment).normalize();

            let added_vel = hit_dir
                * rel_speed
                * consts::curves::BALL_CAR_EXTRA_IMPULSE_FACTOR.get_output(rel_speed)
                * self.mutator_config.ball_hit_extra_force_scale;
            ball_hit_info.extra_hit_vel = added_vel;

            self.ball.velocity_impulse_cache += added_vel;
        }

        car.state.ball_hit_info = Some(ball_hit_info);

        match self.game_mode {
            GameMode::Heatseeker => {
                let can_increase = ball_state.hs_info.time_since_hit
                    > heatseeker::MIN_SPEEDUP_INTERVAL
                    || ball_state.hs_info.y_target_dir == 0.0;
                ball_state.hs_info.y_target_dir = f32::from(car.team == Team::Blue) * 2.0 - 1.0;

                #[allow(clippy::eq_op)]
                if can_increase
                    && ball_state.hs_info.y_target_dir != ball_state.hs_info.y_target_dir
                {
                    ball_state.hs_info.time_since_hit = 0.0;
                    ball_state.hs_info.cur_target_speed = heatseeker::MAX_SPEED.min(
                        ball_state.hs_info.cur_target_speed + heatseeker::TARGET_SPEED_INCREMENT,
                    );
                }
            }
            GameMode::Dropshot => {
                let accumulated_hit_force = &mut ball_state.ds_info.accumulated_hit_force;
                let charge_level = &mut ball_state.ds_info.charge_level;

                let dir_from_car = (ball_state.phys.pos - car.state.phys.pos).normalize();
                let rel_vel_from_car = car.state.phys.vel - ball_state.phys.vel;
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
                    ball_state.ds_info.y_target_dir = f32::from(car.team == Team::Blue) * 2.0 - 1.0;
                }
            }
            _ => {}
        }
    }
}
