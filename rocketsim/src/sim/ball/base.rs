use glam::{Affine3A, Vec3A};

use crate::bullet::dynamics::rigid_body::{ActivationState, CollisionFlags};
use crate::consts::{UU_TO_BT, dropshot, heatseeker};
use crate::{
    BallState, Car, GameMode, MutatorConfig, Team,
    bullet::{
        collision::{
            broadphase::CollisionFilterGroups,
            shapes::{collision_shape::CollisionShapes, sphere_shape::SphereShape},
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
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
            todo!()
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

    pub(crate) fn pre_tick_update(&mut self, game_mode: GameMode) {
        match game_mode {
            GameMode::Heatseeker => todo!(),
            GameMode::Snowday => self.ground_stick_applied = false,
            GameMode::Dropshot | GameMode::Hoops => {
                // launch ball after a short delay on kickoff
                todo!()
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
            rb.lin_vel = rb.lin_vel.normalize() * ball_max_speed_bt;
        }

        if rb.ang_vel.length_squared() > consts::ball::MAX_ANG_SPEED * consts::ball::MAX_ANG_SPEED {
            rb.ang_vel = rb.ang_vel.normalize() * consts::ball::MAX_ANG_SPEED;
        }

        self.state.phys.vel = rb.lin_vel * consts::BT_TO_UU;
        self.state.phys.ang_vel = rb.ang_vel;

        let trans = *rb.get_world_trans();
        self.state.phys.pos = trans.translation * consts::BT_TO_UU;
        self.state.phys.rot_mat = trans.matrix3;

        self.state.hit_last_tick = false;
    }

    pub(crate) fn on_hit(
        &mut self,
        car: &mut Car,
        game_mode: GameMode,
        mutator_config: &MutatorConfig,
    ) {
        let car_forward = car.state.phys.rot_mat.x_axis;
        let rel_pos = self.state.phys.pos - car.state.phys.pos;
        let rel_vel = self.state.phys.vel - car.state.phys.vel;

        let rel_speed = rel_vel
            .length()
            .min(consts::ball::car_hit_impulse::MAX_DELTA_VEL_UU);
        if rel_speed > 0.0 && self.state.hit_last_tick {
            let extra_z_scale = game_mode == GameMode::Hoops
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
                * mutator_config.ball_hit_extra_force_scale;
            self.vel_impulse_cache += added_vel;
        }

        match game_mode {
            GameMode::Heatseeker => {
                let can_increase = self.state.hs_info.time_since_hit
                    > heatseeker::MIN_SPEEDUP_INTERVAL
                    || self.state.hs_info.y_target_dir == 0.0;
                self.state.hs_info.y_target_dir = f32::from(car.team == Team::Blue) * 2.0 - 1.0;

                #[allow(clippy::eq_op)]
                if can_increase
                    && self.state.hs_info.y_target_dir != self.state.hs_info.y_target_dir
                {
                    self.state.hs_info.time_since_hit = 0.0;
                    self.state.hs_info.cur_target_speed = heatseeker::MAX_SPEED.min(
                        self.state.hs_info.cur_target_speed + heatseeker::TARGET_SPEED_INCREMENT,
                    );
                }
            }
            GameMode::Dropshot => {
                let accumulated_hit_force = &mut self.state.ds_info.accumulated_hit_force;
                let charge_level = &mut self.state.ds_info.charge_level;

                let dir_from_car = (self.state.phys.pos - car.state.phys.pos).normalize();
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
                    self.state.ds_info.y_target_dir = f32::from(car.team == Team::Blue) * 2.0 - 1.0;
                }
            }
            _ => {}
        };
    }
}
