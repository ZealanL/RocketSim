use super::ArenaContactTracker;
use crate::ArenaEvent::{BallHitWorld, CarPickupBoost};
use crate::bullet::dynamics::rigid_body::ActivationState;
use crate::consts::{TICK_RATE, TICK_TIME};
use crate::sim::ArenaEvent::CarHitBall;
use crate::sim::arena::ArenaEventList;
use crate::sim::{ArenaEvent, Ball, BoostPad, CarHitBallEvent, CarHitCarEvent, CarHitWorldEvent};
use crate::{
    ARENA_COLLISION_MESH_FILES, ARENA_COLLISION_SHAPES, ArenaConfig, ArenaMemWeightMode,
    ArenaState, BallHitWorldEvent, BoostPadConfig, BoostPadGrid, BoostPadState, Car, CarBodyConfig,
    CarControls, CarInfo, CarPickupBoostEvent, CarState, GameMode, MutatorConfig, PhysState, Team,
    bullet::{
        collision::{
            broadphase::{GridBroadphase, HashedOverlappingPairCache},
            dispatch::collision_dispatcher::CollisionDispatcher,
            narrowphase::manifold_point::ManifoldPoint,
            shapes::{collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape},
        },
        dynamics::{
            constraint_solver::seq_impulse_constraint_solver::SeqImpulseConstraintSolver,
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{RigidBody, RigidBodyConstructionInfo},
        },
    },
    consts,
    consts::{BT_TO_UU, UU_TO_BT},
    sim::{BallState, DemoMode, UserInfoTypes, collision_masks::CollisionMasks},
};
use arrayvec::ArrayVec;
use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};
use std::{f32::consts::PI, iter::repeat_n, mem};

#[cfg(feature = "vis")]
use crate::vis::VisInst;

pub struct Arena {
    pub(crate) bullet_world: DiscreteDynamicsWorld,
    pub(crate) rng: Rng,
    config: ArenaConfig,

    pub(crate) ball: Ball,
    pub(crate) cars: Vec<Car>,
    pub(crate) tick_count: u64,
    pub(crate) game_mode: GameMode,
    pub(crate) mutator_config: MutatorConfig,
    pub(crate) boost_pad_grid: BoostPadGrid,
    pub(crate) contact_tracker: ArenaContactTracker,
    pub(crate) events: ArenaEventList,

    #[cfg(feature = "vis")]
    vis_inst: Option<VisInst>,
}

impl Arena {
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(game_mode, ArenaConfig::DEFAULT)
    }

    pub fn new_with_config(game_mode: GameMode, config: ArenaConfig) -> Self {
        let mutator_config = MutatorConfig::new(game_mode);

        let collision_dispatcher = CollisionDispatcher::default();
        let constraint_solver = SeqImpulseConstraintSolver::default();
        let overlapping_pair_cache = HashedOverlappingPairCache::default();

        let (cell_size_multiplier, initial_handle_size) = match config.mem_weight_mode {
            ArenaMemWeightMode::Light => (3.0, 1),
            ArenaMemWeightMode::Heavy => (1.0, 8),
        };

        let broadphase = GridBroadphase::new(
            config.min_pos * UU_TO_BT,
            config.max_pos * UU_TO_BT,
            config.max_aabb_len * UU_TO_BT * cell_size_multiplier,
            initial_handle_size,
            overlapping_pair_cache,
        );

        let mut bullet_world =
            DiscreteDynamicsWorld::new(collision_dispatcher, broadphase, constraint_solver);
        bullet_world.set_gravity(mutator_config.gravity * UU_TO_BT);

        if game_mode != GameMode::TheVoid {
            Self::setup_arena_collision_shapes(&mut bullet_world, game_mode);
        }

        let ball = Ball::new(
            game_mode,
            &mut bullet_world,
            &mutator_config,
            config.no_ball_rot,
        );

        let boost_pad_grid = {
            let mut boost_pad_configs = Vec::new();
            if game_mode != GameMode::TheVoid && game_mode != GameMode::Dropshot {
                if config.use_custom_boost_pads {
                    boost_pad_configs.extend_from_slice(&config.custom_boost_pads);
                } else {
                    let small_pad_locs = consts::boost_pads::get_locations(game_mode, false);
                    let big_pad_locs = consts::boost_pads::get_locations(game_mode, true);
                    boost_pad_configs.reserve(small_pad_locs.len() + big_pad_locs.len());

                    for small_pos in small_pad_locs {
                        boost_pad_configs.push(BoostPadConfig {
                            pos: *small_pos,
                            is_big: false,
                        });
                    }
                    for big_pos in big_pad_locs {
                        boost_pad_configs.push(BoostPadConfig {
                            pos: *big_pos,
                            is_big: true,
                        });
                    }
                }
            }

            BoostPadGrid::new(&boost_pad_configs, &mutator_config)
        };

        let rng = config.rng_seed.map_or_else(Rng::new, Rng::with_seed);

        Self {
            rng,
            config,
            ball,
            game_mode,
            boost_pad_grid,
            mutator_config,
            tick_count: 0,
            cars: Vec::with_capacity(6),
            bullet_world,

            contact_tracker: ArenaContactTracker::new(),
            events: ArenaEventList::new(),

            #[cfg(feature = "vis")]
            vis_inst: None,
        }
    }

    pub const fn get_config(&self) -> &ArenaConfig {
        &self.config
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
        group: u8,
        mask: u8,
    ) {
        let mut rb_info = RigidBodyConstructionInfo::new(0.0, shape, false);
        rb_info.restitution = consts::arena::BASE_COEFS.restitution;
        rb_info.friction = consts::arena::BASE_COEFS.friction;
        rb_info.start_world_trans.translation = pos_bt;

        let shape_rb = RigidBody::new(rb_info);
        if (group | mask) != 0 {
            bullet_world.add_rigid_body(shape_rb, group, mask);
        } else {
            bullet_world.add_rigid_body_default(shape_rb);
        }
    }

    fn setup_arena_collision_shapes(bullet_world: &mut DiscreteDynamicsWorld, game_mode: GameMode) {
        debug_assert!(game_mode != GameMode::TheVoid);

        let collision_shapes = ARENA_COLLISION_SHAPES.read().unwrap();
        let collision_meshes = &collision_shapes
            .as_ref()
            .expect("Arena collision shapes are uninitialized - please call init(..) first.")
            [&game_mode];
        assert!(
            !collision_meshes.is_empty(),
            "No arena meshes found for the game mode {game_mode:?}"
        );

        for mesh in collision_meshes {
            let is_hoops_net = if game_mode == GameMode::Hoops {
                todo!()
            } else {
                false
            };

            let mask = if is_hoops_net {
                CollisionMasks::HoopsNet as u8
            } else {
                0
            };

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::TriangleMesh(mesh.clone()),
                Vec3A::ZERO,
                mask,
                mask,
            );
        }

        drop(collision_shapes);

        let arena_aabb = consts::arena::get_aabb(game_mode);

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A, mask: u8| {
            debug_assert!(normal.is_normalized());
            let pos_bt = pos_uu * UU_TO_BT;
            let trans = Affine3A {
                matrix3: Mat3A::IDENTITY,
                translation: pos_bt,
            };

            let plane_shape = StaticPlaneShape::new(trans, normal);

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::StaticPlane(plane_shape),
                pos_bt,
                mask,
                mask,
            );
        };

        let floor_mask = if game_mode == GameMode::Dropshot {
            CollisionMasks::DropshotFloor as u8
        } else {
            0
        };

        // Floor
        add_plane(Vec3A::new(0.0, 0.0, arena_aabb.min.z), Vec3A::Z, floor_mask);

        // Ceiling
        add_plane(Vec3A::new(0.0, 0.0, arena_aabb.max.z), Vec3A::NEG_Z, 0);

        match game_mode {
            GameMode::Hoops => {
                // Y walls
                add_plane(
                    Vec3A::new(0.0, arena_aabb.min.y, arena_aabb.center().z),
                    Vec3A::Y,
                    0,
                );

                add_plane(
                    Vec3A::new(0.0, arena_aabb.min.z, arena_aabb.center().z),
                    Vec3A::NEG_Y,
                    0,
                );
            }
            GameMode::Dropshot => {
                // Add tiles
                todo!()
            }
            _ => {
                // Side walls
                add_plane(
                    Vec3A::new(arena_aabb.min.x, 0.0, arena_aabb.center().z),
                    Vec3A::X,
                    0,
                );
                add_plane(
                    Vec3A::new(arena_aabb.max.x, 0.0, arena_aabb.center().z),
                    Vec3A::NEG_X,
                    0,
                );
            }
        }
    }

    fn ball_within_hoops_goal_xy_margin_eq(x: f32, y: f32) -> f32 {
        const SCALE_Y: f32 = 0.9;
        const OFFSET_Y: f32 = 2770.0;
        const RADIUS_SQ: f32 = 716.0 * 716.0;

        let dy = y.abs() * SCALE_Y - OFFSET_Y;
        let dist_sq = x * x + dy * dy;
        dist_sq - RADIUS_SQ
    }

    pub fn is_ball_scored(&self) -> bool {
        let ball_pos = self.bullet_world.bodies()[self.ball.rigid_body_idx]
            .get_world_trans()
            .translation
            * BT_TO_UU;

        match self.game_mode {
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday => {
                ball_pos.y.abs()
                    > self.mutator_config.goal_base_threshold_y + self.mutator_config.ball_radius
            }
            GameMode::Hoops => {
                if ball_pos.z < consts::goal::HOOPS_GOAL_SCORE_THRESHOLD_Z {
                    Self::ball_within_hoops_goal_xy_margin_eq(ball_pos.x, ball_pos.y) < 0.0
                } else {
                    false
                }
            }
            GameMode::Dropshot => ball_pos.z < -self.mutator_config.ball_radius * 1.75,
            GameMode::TheVoid => false,
        }
    }

    pub fn reset_to_random_kickoff(&mut self) {
        let game_mode = self.game_mode;
        let kickoff_locs = consts::car::spawn::get_kickoff_spawn_locations(game_mode);
        let respawn_locs = consts::car::spawn::get_respawn_locations(game_mode);

        let mut kickoff_order_perm = ArrayVec::<usize, 5>::new();
        kickoff_order_perm.extend(0..kickoff_locs.len());
        self.rng.shuffle(&mut kickoff_order_perm);

        let mut num_blue_cars = 0;
        let mut num_orange_cars = 0;

        for car in &mut self.cars {
            if car.team == Team::Blue {
                num_blue_cars += 1;
            } else {
                num_orange_cars += 1;
            }
        }

        let mut num_cars_at_respawn_pos = ArrayVec::<usize, 4>::new();
        num_cars_at_respawn_pos.extend(repeat_n(0, 4));

        let kickoff_pos_amount = num_blue_cars.max(num_orange_cars);
        for i in 0..kickoff_pos_amount {
            let spawn_pos = if i < kickoff_locs.len() {
                kickoff_locs[kickoff_order_perm[i]]
            } else {
                const CAR_SPAWN_EXTRA_OFFSET_Y: f32 = 250.0;

                let respawn_pos_idx = (i - kickoff_locs.len()) % respawn_locs.len();
                let mut pos = respawn_locs[respawn_pos_idx];
                pos.y += CAR_SPAWN_EXTRA_OFFSET_Y * num_cars_at_respawn_pos[respawn_pos_idx] as f32;
                num_cars_at_respawn_pos[respawn_pos_idx] += 1;

                pos
            };

            let mut spawn_state = CarState {
                phys: PhysState {
                    pos: Vec3A::new(spawn_pos.x, spawn_pos.y, consts::car::spawn::SPAWN_Z),
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                },
                boost: self.mutator_config.car_spawn_boost_amount,
                is_on_ground: true,
                ..Default::default()
            };

            for cur_team in Team::ALL {
                let is_blue = cur_team == Team::Blue;

                let mut team_car_indices = Vec::with_capacity(self.cars.len());
                for car in &self.cars {
                    if car.team == cur_team {
                        team_car_indices.push(car.idx);
                    }
                }

                if team_car_indices.len() <= i {
                    continue;
                }

                let car_idx = team_car_indices[i];

                spawn_state.phys.rot_mat = Mat3A::from_euler(
                    EulerRot::ZYX,
                    if is_blue {
                        spawn_pos.yaw_ang
                    } else {
                        spawn_state.phys.pos *= Vec3A::new(-1.0, -1.0, 1.0);
                        spawn_pos.yaw_ang + if is_blue { 0.0 } else { PI }
                    },
                    0.0,
                    0.0,
                );

                let car = &mut self.cars[car_idx];
                let rb = &mut self.bullet_world.bodies_mut()[car.rigid_body_idx];
                car.set_state(rb, &spawn_state);
            }
        }

        let mut ball_state = BallState::DEFAULT;
        match self.game_mode {
            GameMode::Heatseeker => {
                let next_rand = self.rng.bool();
                let y_sign = f32::from(i8::from(next_rand) * 2 - 1);
                let scale = Vec3A::new(1.0, y_sign, 1.0);
                ball_state.phys.pos = consts::heatseeker::BALL_START_POS * scale;
                ball_state.phys.vel = consts::heatseeker::BALL_START_VEL * scale;
            }
            GameMode::Snowday => {
                ball_state.phys.vel.z = f32::EPSILON;
            }
            _ => {}
        }

        self.set_ball_state(ball_state);

        self.boost_pad_grid.reset();

        // TODO: Reset tile states
    }

    /// Creates and adds a car to the arena, returning the index of the car in the cars vector
    pub fn add_car(&mut self, team: Team, config: CarBodyConfig) -> usize {
        let idx = self.cars.len();

        let mut car = Car::new(
            idx,
            team,
            &mut self.bullet_world,
            &self.mutator_config,
            config,
        );
        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.game_mode,
            self.mutator_config.car_spawn_boost_amount,
        );

        self.bullet_world.bodies_mut()[car.rigid_body_idx].user_pointer = idx;
        self.cars.push(car);
        idx
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) -> &[ArenaEvent] {
        self.events.clear();

        // Update ball activation
        {
            let ball_rb = &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx];
            let should_sleep =
                ball_rb.lin_vel.length_squared() == 0.0 && ball_rb.ang_vel.length_squared() == 0.0;

            ball_rb.set_activation_state(if should_sleep {
                ActivationState::Sleeping
            } else {
                ActivationState::Active
            });
        }

        for car in &mut self.cars {
            car.pre_tick_update(
                &mut self.bullet_world,
                &mut self.rng,
                self.game_mode,
                &self.mutator_config,
            );
        }

        self.ball.pre_tick_update(self.game_mode);

        self.bullet_world
            .step_simulation(TICK_TIME, &mut self.contact_tracker);

        let contact_count = self.contact_tracker.num_records();
        for idx in 0..contact_count {
            let contact = *self.contact_tracker.get_record(idx);
            let [body_a, body_b] = self
                .bullet_world
                .bodies_mut()
                .get_disjoint_mut([contact.rb_idx_a, contact.rb_idx_b])
                .unwrap();

            let user_pointer_a = body_a.user_pointer;
            let user_pointer_b = body_b.user_pointer;

            if contact.user_idx_a == UserInfoTypes::Car {
                match contact.user_idx_b {
                    UserInfoTypes::Ball => {
                        self.on_car_ball_collision(
                            user_pointer_a,
                            &contact.manifold_point,
                            contact.is_swap,
                        );
                    }
                    UserInfoTypes::Car => {
                        self.on_car_car_collision(
                            user_pointer_a,
                            user_pointer_b,
                            &contact.manifold_point,
                        );
                    }
                    _ => self.on_car_world_collision(user_pointer_a, &contact.manifold_point),
                }
            } else if contact.user_idx_a == UserInfoTypes::Ball {
                self.on_ball_world_collision(&contact.manifold_point);
            }
        }

        self.contact_tracker.clear_records();

        for car in &mut self.cars {
            let rb = &mut self.bullet_world.bodies_mut()[car.rigid_body_idx];
            car.post_tick_update(rb);
            car.finish_physics_tick(rb);

            let collected_pad_op = self.boost_pad_grid.maybe_give_car_boost(
                &mut car.state,
                &self.mutator_config,
                self.tick_count,
            );

            if let Some(collected_pad_idx) = collected_pad_op {
                self.events.push(CarPickupBoost(CarPickupBoostEvent {
                    car_idx: car.idx,
                    boost_pad_idx: collected_pad_idx,
                }));
            }
        }

        let ball_rb = &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx];
        self.ball.finish_physics_tick(ball_rb, &self.mutator_config);

        if self.game_mode == GameMode::Dropshot {
            todo!("Dropshot tile state sync")
        }

        self.tick_count += 1;

        #[cfg(feature = "vis")]
        if self.vis_inst.is_some() {
            let arena_state = self.get_arena_state();
            self.vis_inst
                .as_mut()
                .unwrap()
                .update(&arena_state, TICK_TIME);
        };

        self.get_last_step_events()
    }

    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
    }

    #[inline]
    pub const fn game_mode(&self) -> GameMode {
        self.game_mode
    }

    #[inline]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.mutator_config
    }

    pub fn set_ball_state(&mut self, ball_state: BallState) {
        self.ball.set_state(
            &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx],
            ball_state,
        );
    }

    pub fn get_ball_state(&self) -> &BallState {
        &self.ball.state
    }

    #[inline]
    pub const fn cars(&self) -> &Vec<Car> {
        &self.cars
    }

    #[inline]
    pub const fn num_cars(&self) -> usize {
        self.cars.len()
    }

    pub fn get_car_info(&self, car_idx: usize) -> &CarInfo {
        &self.cars[car_idx].info
    }

    pub fn get_car_state(&self, car_idx: usize) -> &CarState {
        self.cars[car_idx].get_state()
    }

    pub fn get_car_controls(&self, car_idx: usize) -> &CarControls {
        &self.cars[car_idx].state.controls
    }

    pub fn get_car_info_and_state(&self, car_idx: usize) -> (&CarInfo, &CarState) {
        let car = &self.cars[car_idx];
        (&car.info, &car.state)
    }

    pub fn set_car_state(&mut self, car_idx: usize, state: CarState) {
        let car = &mut self.cars[car_idx];

        car.set_state(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &state,
        );
    }

    pub fn set_car_controls(&mut self, car_idx: usize, controls: CarControls) {
        self.cars[car_idx].state.controls = controls
    }

    pub fn respawn_car(&mut self, car_idx: usize) {
        let car = &mut self.cars[car_idx];

        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.game_mode,
            self.mutator_config.car_spawn_boost_amount,
        );
    }

    pub fn get_boost_pad_state(&self, idx: usize) -> BoostPadState {
        let pad = self.boost_pads()[idx];

        let cooldown = if let Some(gave_boost_tick) = pad.gave_boost_tick_count {
            let max_cooldown = pad.max_cooldown;
            let time_since = ((self.tick_count() - gave_boost_tick) as f32) * TICK_TIME;
            (max_cooldown - time_since).max(0.0)
        } else {
            0.0
        };

        BoostPadState { cooldown }
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        let tick_count = self.tick_count;
        let pad = &mut self.boost_pad_grid.all_pads[idx];
        if state.cooldown > 0.0 {
            let time_since_pickup = (pad.max_cooldown - state.cooldown).max(0.0);
            let ticks_since_pickup = (time_since_pickup * TICK_RATE).round() as u64;
            pad.gave_boost_tick_count = Some(tick_count - ticks_since_pickup);
        } else {
            self.boost_pad_grid.all_pads[idx].gave_boost_tick_count = None;
        }
    }

    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }

    pub(crate) fn boost_pads(&self) -> &[BoostPad] {
        &self.boost_pad_grid.all_pads
    }

    pub fn num_boost_pads(&self) -> usize {
        self.boost_pads().len()
    }

    pub fn get_all_boost_pad_states(&self) -> Vec<BoostPadState> {
        (0..self.num_boost_pads())
            .map(|i| self.get_boost_pad_state(i))
            .collect()
    }

    pub fn get_all_boost_pad_configs(&self) -> Vec<BoostPadConfig> {
        (0..self.num_boost_pads())
            .map(|i| *self.get_boost_pad_config(i))
            .collect()
    }

    pub fn get_arena_state(&self) -> ArenaState {
        let car_infos = self.cars.iter().map(|c| c.info).collect::<Vec<_>>();
        let car_states = self.cars.iter().map(|c| c.state).collect::<Vec<_>>();
        let ball_state = *self.get_ball_state();
        let boost_pad_states = self.get_all_boost_pad_states();
        let boost_pad_configs = self.get_all_boost_pad_configs();
        ArenaState {
            game_mode: self.game_mode,
            car_infos,
            car_states,
            ball_state,
            boost_pad_states,
            boost_pad_configs,
        }
    }

    /// Returns the events generated during the last stepped tick
    pub fn get_last_step_events(&self) -> &[ArenaEvent] {
        self.events.events()
    }

    #[cfg(feature = "vis")]
    pub fn get_vis_enabled(&self) -> bool {
        self.vis_inst.is_some()
    }

    #[cfg(feature = "vis")]
    pub fn set_vis_enabled(&mut self, vis_enabled: bool) {
        if vis_enabled && self.vis_inst.is_none() {
            // Create visualizer

            let collision_mesh_files = ARENA_COLLISION_MESH_FILES.read().unwrap();
            let game_mode_mesh_files = &collision_mesh_files.as_ref().unwrap()[&self.game_mode];

            self.vis_inst = Some(VisInst::new(
                self.game_mode,
                game_mode_mesh_files.as_slice(),
            ));
        } else if !vis_enabled && self.vis_inst.is_some() {
            unimplemented!(); // TODO: Stop render loop properly
            // self.vis_inst = None;
        }
    }
}

impl Arena {
    fn on_ball_world_collision(&mut self, manifold_point: &ManifoldPoint) {
        let contact_point = manifold_point.pos_world_on_b * BT_TO_UU;
        let contact_normal = manifold_point.normal_world_on_b;

        self.events.push(BallHitWorld(BallHitWorldEvent {
            contact_point,
            contact_normal,
        }))
    }

    fn on_car_ball_collision(
        &mut self,
        car_idx: usize,
        manifold_point: &ManifoldPoint,
        ball_is_body_a: bool,
    ) {
        self.ball.on_hit(
            &mut self.cars[car_idx],
            self.game_mode,
            &self.mutator_config,
            self.tick_count,
        );

        let contact_point = if ball_is_body_a {
            manifold_point.pos_world_on_a
        } else {
            manifold_point.pos_world_on_b
        } * BT_TO_UU;

        // TODO: Somewhat hacky
        let extra_hit_vel = self.ball.vel_impulse_cache;

        self.events.push(CarHitBall(CarHitBallEvent {
            car_idx,
            contact_point,
            extra_hit_vel,
        }));
    }

    fn on_car_world_collision(&mut self, car_idx: usize, manifold_point: &ManifoldPoint) {
        self.events.push(ArenaEvent::CarHitWorld(CarHitWorldEvent {
            car_idx,
            contact_point: manifold_point.pos_world_on_b * BT_TO_UU,
            contact_normal: manifold_point.normal_world_on_b,
        }));
        self.cars[car_idx].state.world_contact_normal = Some(manifold_point.normal_world_on_b);
    }

    fn on_car_car_collision(
        &mut self,
        car_1_idx: usize,
        car_2_idx: usize,
        manifold_point: &ManifoldPoint,
    ) {
        let Ok(both_cars) = self.cars.get_disjoint_mut([car_1_idx, car_2_idx]) else {
            panic!(
                "on_car_car_collision() called with invalid or duplicate car indices: {car_1_idx}, {car_2_idx}"
            );
        };

        let (mut attacker, mut victim) = both_cars.into();

        // NOTE: Checking the victim first, because in many use-cases, repeat-demo-victims are more likely
        if victim.state.is_demoed || attacker.state.is_demoed {
            return;
        }

        // Test collision both ways
        for is_swapped in [false, true] {
            let mut attacker_idx = car_1_idx;
            let mut victim_idx = car_2_idx;
            if is_swapped {
                mem::swap(&mut attacker, &mut victim);
                mem::swap(&mut attacker_idx, &mut victim_idx);
            }

            let attacker_state = &attacker.state;
            let victim_state = &victim.state;

            if attacker_state.bump_cooldown_timer > 0.0 {
                // In cooldown
                continue;
            }

            let delta_pos = victim_state.phys.pos - attacker_state.phys.pos;
            if attacker_state.phys.vel.dot(delta_pos) < 0.0 {
                // Moving away from the other car
                continue;
            }

            let vel_dir = attacker_state.phys.vel.normalize_or_zero();
            let dir_to_victim = delta_pos.normalize();

            let speed_towards_other_car = attacker_state.phys.vel.dot(dir_to_victim);
            let other_car_away_speed = victim_state.phys.vel.dot(vel_dir);
            if speed_towards_other_car <= other_car_away_speed {
                // Going towards other car slower than they're going away
                continue;
            }

            if self.mutator_config.bump_requires_front_hit {
                let local_point_x = if is_swapped {
                    manifold_point.local_point_b
                } else {
                    manifold_point.local_point_a
                }
                .x * BT_TO_UU;

                let hit_with_bumper = local_point_x > consts::car::bump::MIN_FORWARD_DIST;
                if !hit_with_bumper {
                    // Didn't hit with bumper
                    continue;
                }
            }

            let mut is_demo = match self.mutator_config.demo_mode {
                DemoMode::OnContact => true,
                DemoMode::Disabled => false,
                DemoMode::Normal => attacker_state.is_supersonic,
            };
            if is_demo && !self.mutator_config.enable_team_demos {
                is_demo = attacker.team != victim.team;
            }

            if is_demo {
                victim.demolish(self.mutator_config.respawn_delay);
            } else {
                let ground_hit = victim_state.is_on_ground;
                let base_scale = if ground_hit {
                    consts::curves::BUMP_VEL_AMOUNT_GROUND
                } else {
                    consts::curves::BUMP_VEL_AMOUNT_AIR
                }
                .get_output(speed_towards_other_car);

                let hit_up_dir = if victim_state.is_on_ground {
                    victim_state.phys.rot_mat.z_axis
                } else {
                    Vec3A::Z
                };

                let upward_vel_curve = &consts::curves::BUMP_UPWARD_VEL_AMOUNT;
                let upward_force = upward_vel_curve.get_output(speed_towards_other_car)
                    * self.mutator_config.bump_force_scale;
                let bump_impulse = (vel_dir * base_scale) + (hit_up_dir * upward_force);

                victim.vel_impulse_cache += bump_impulse;
            }

            let contact_point = if is_swapped {
                manifold_point.pos_world_on_b
            } else {
                manifold_point.pos_world_on_a
            } * BT_TO_UU;

            self.events.push(ArenaEvent::CarHitCar(CarHitCarEvent {
                bumper_car_idx: attacker_idx,
                victim_car_idx: victim_idx,
                contact_point,
                is_demo,
            }));
        }
    }
}
