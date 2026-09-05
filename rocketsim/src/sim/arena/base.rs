use std::{any::Any, f32::consts::PI, iter::repeat_n, mem};

use arrayvec::ArrayVec;
use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};
#[cfg(debug_assertions)]
use indexmap::IndexMap;

use super::ArenaContactTracker;
use crate::{
    ARENA_COLLISION_SHAPES, ArenaConfig,
    ArenaEvent::{BallHitWorld, CarPickupBoost},
    ArenaMemWeightMode, ArenaState, BallHitWorldEvent, BoostPadConfig, BoostPadGrid, BoostPadState,
    Car, CarBodyConfig, CarControls, CarInfo, CarPickupBoostEvent, CarState, GameMode,
    MutatorConfig, PhysState, RaycastHitInfo, RaycastQuery, RaycastResult, Team, TileDamageState,
    TileStates,
    bullet::{
        collision::{
            broadphase::{CollisionFilterGroups, GridBroadphase},
            dispatch::quad_ray_callbacks::{ClosestQuadRayResultCallback, QuadRayResultCallback},
            narrowphase::manifold_point::ManifoldPoint,
            shapes::{collision_shape::CollisionShapes, static_plane_shape::StaticPlaneShape},
        },
        dynamics::{
            discrete_dynamics_world::DiscreteDynamicsWorld,
            rigid_body::{CollisionFlags, RigidBody, RigidBodyConstructionInfo},
        },
    },
    consts::{self, BT_TO_UU, TICK_RATE, TICK_TIME, UU_TO_BT},
    make_tile_shapes,
    shared::quantize,
    sim::{
        ArenaEvent, Ball, BallState, BoostPad, CarHitBallEvent, CarHitCarEvent, CarHitWorldEvent,
        DemoMode, UserInfoTypes, arena::ArenaEventList,
    },
};

pub trait Vis: Send + Sync + Any {
    fn update(&mut self, arena_state: &ArenaState, dt: f32);
}

pub struct Arena {
    pub(crate) bullet_world: DiscreteDynamicsWorld,
    config: ArenaConfig,

    pub(crate) ball: Ball,
    pub(crate) cars: Vec<Car>,
    pub(crate) tick_count: u64,
    pub(crate) boost_pad_grid: Option<BoostPadGrid>,
    pub(crate) tile_states: Option<TileStates>,
    pub(crate) contact_tracker: ArenaContactTracker,
    pub(crate) events: ArenaEventList,

    pub rng: Rng,
    pub vis: Option<Box<dyn Vis>>,
}

impl Arena {
    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(ArenaConfig::new(game_mode))
    }

    pub fn new_with_config(config: ArenaConfig) -> Self {
        let (cell_size, initial_handle_size) = match config.mem_weight_mode {
            ArenaMemWeightMode::Light => ((config.max_pos - config.min_pos).max_element(), 1),
            ArenaMemWeightMode::Balanced => (config.max_aabb_len * 3.0, 1),
            ArenaMemWeightMode::Heavy => (config.max_aabb_len, 8),
        };

        let broadphase = GridBroadphase::new(
            config.min_pos * UU_TO_BT,
            config.max_pos * UU_TO_BT,
            cell_size * UU_TO_BT,
            initial_handle_size,
        );

        let mut bullet_world =
            DiscreteDynamicsWorld::new(broadphase, config.mutators.gravity * UU_TO_BT);

        if config.game_mode != GameMode::TheVoid {
            Self::setup_arena_collision_shapes(&mut bullet_world, config.game_mode);
        }

        let ball = Ball::new(
            config.game_mode,
            &mut bullet_world,
            &config.mutators,
            config.no_ball_rot,
        );

        let boost_pad_grid =
            if config.game_mode != GameMode::TheVoid && config.game_mode != GameMode::Dropshot {
                let mut boost_pad_configs = Vec::new();

                if let Some(custom_boost_pads) = config.custom_boost_pads.as_ref() {
                    boost_pad_configs.extend_from_slice(custom_boost_pads);
                } else {
                    let small_pad_locs = consts::boost_pads::get_locations(config.game_mode, false);
                    let big_pad_locs = consts::boost_pads::get_locations(config.game_mode, true);
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

                Some(BoostPadGrid::new(&boost_pad_configs, &config.mutators))
            } else {
                None
            };

        let tile_states = if config.game_mode == GameMode::Dropshot {
            Some(TileStates::DEFAULT)
        } else {
            None
        };

        let rng = config.rng_seed.map_or_else(Rng::new, Rng::with_seed);

        Self {
            rng,
            config,
            ball,
            boost_pad_grid,
            tick_count: 0,
            cars: Vec::with_capacity(6),
            bullet_world,
            tile_states,

            contact_tracker: ArenaContactTracker::new(),
            events: ArenaEventList::new(),

            vis: None,
        }
    }

    #[must_use]
    pub const fn get_config(&self) -> &ArenaConfig {
        &self.config
    }

    fn add_static_collision_shape(
        bullet_world: &mut DiscreteDynamicsWorld,
        shape: CollisionShapes,
        pos_bt: Vec3A,
        group: Option<u8>,
    ) -> usize {
        let mut rb_info = RigidBodyConstructionInfo::new(0.0, shape);
        rb_info.restitution = consts::arena::BASE_COEFS.restitution;
        rb_info.friction = consts::arena::BASE_COEFS.friction;
        rb_info.start_world_trans.translation = pos_bt;

        let shape_rb = RigidBody::new(rb_info);
        if let Some(group) = group {
            bullet_world.add_rigid_body(
                shape_rb,
                group | CollisionFilterGroups::Static,
                group ^ CollisionFilterGroups::Static,
            )
        } else {
            bullet_world.add_rigid_body_default(shape_rb)
        }
    }

    fn setup_arena_collision_shapes(bullet_world: &mut DiscreteDynamicsWorld, game_mode: GameMode) {
        debug_assert!(game_mode != GameMode::TheVoid);

        let mesh_game_mode = match game_mode {
            GameMode::Heatseeker | GameMode::Snowday => GameMode::Soccar,
            _ => game_mode,
        };
        let collision_shapes = ARENA_COLLISION_SHAPES.read().unwrap();
        let collision_meshes = &collision_shapes
            .as_ref()
            .expect("Arena collision shapes are uninitialized - please call init(..) first.")
            [&mesh_game_mode];
        assert!(
            !collision_meshes.is_empty(),
            "No arena meshes found for the game mode {game_mode:?}"
        );

        for mesh in collision_meshes {
            let is_hoops_net = if game_mode == GameMode::Hoops {
                // Detect net mesh and disable car collision
                mesh.shape.get_mesh_interface().get_total_num_faces() == 798
            } else {
                false
            };

            let mask = if is_hoops_net {
                Some(CollisionFilterGroups::HoopsNet as u8)
            } else {
                None
            };

            Self::add_static_collision_shape(
                bullet_world,
                CollisionShapes::TriangleMesh(mesh.shape.clone()),
                mesh.translation,
                mask,
            );
        }

        drop(collision_shapes);

        let arena_aabb = consts::arena::get_aabb(game_mode);

        let mut add_plane = |pos_uu: Vec3A, normal: Vec3A, mask: Option<u8>| {
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
            );
        };

        let floor_mask = if game_mode == GameMode::Dropshot {
            Some(CollisionFilterGroups::DropshotFloor as u8)
        } else {
            None
        };

        // Floor
        add_plane(Vec3A::new(0.0, 0.0, arena_aabb.min.z), Vec3A::Z, floor_mask);

        // Ceiling
        add_plane(Vec3A::new(0.0, 0.0, arena_aabb.max.z), Vec3A::NEG_Z, None);

        if game_mode != GameMode::Dropshot {
            // Side walls
            add_plane(
                Vec3A::new(arena_aabb.min.x, 0.0, arena_aabb.max.z / 2.0),
                Vec3A::X,
                None,
            );
            add_plane(
                Vec3A::new(arena_aabb.max.x, 0.0, arena_aabb.max.z / 2.0),
                Vec3A::NEG_X,
                None,
            );
        }

        match game_mode {
            GameMode::Hoops => {
                // Y walls
                add_plane(
                    Vec3A::new(0.0, arena_aabb.min.y, arena_aabb.max.z / 2.0),
                    Vec3A::Y,
                    None,
                );

                add_plane(
                    Vec3A::new(0.0, arena_aabb.max.y, arena_aabb.max.z / 2.0),
                    Vec3A::NEG_Y,
                    None,
                );
            }
            GameMode::Dropshot => {
                for (i, tile) in make_tile_shapes().enumerate() {
                    // Shift down so the collision doesn't peek through the floor
                    let pos = Vec3A::new(0.0, 0.0, -tile.get_margin());

                    let rb_idx = Self::add_static_collision_shape(
                        bullet_world,
                        CollisionShapes::ConvexHull(tile),
                        pos,
                        Some(CollisionFilterGroups::DropshotTile as u8),
                    );

                    let rb = &mut bullet_world.bodies_mut()[rb_idx];
                    rb.user_idx = UserInfoTypes::DropshotTile;
                    rb.user_pointer = i;
                }
            }
            _ => {}
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

    #[must_use]
    pub fn is_ball_scored(&self) -> bool {
        let ball_pos = self.bullet_world.bodies()[self.ball.rigid_body_idx]
            .get_world_trans()
            .translation
            * BT_TO_UU;

        match self.config.game_mode {
            GameMode::Soccar | GameMode::Heatseeker | GameMode::Snowday => {
                ball_pos.y.abs()
                    > self.config.mutators.goal_base_threshold_y + self.config.mutators.ball_radius
            }
            GameMode::Hoops => {
                if ball_pos.z < consts::goal::HOOPS_GOAL_SCORE_THRESHOLD_Z {
                    Self::ball_within_hoops_goal_xy_margin_eq(ball_pos.x, ball_pos.y) < 0.0
                } else {
                    false
                }
            }
            GameMode::Dropshot => ball_pos.z < -self.config.mutators.ball_radius * 1.75,
            GameMode::TheVoid => false,
        }
    }

    pub fn reset_to_random_kickoff(&mut self, rng_seed: Option<u64>) {
        let kickoff_locs = consts::car::spawn::get_kickoff_spawn_locations(self.config.game_mode);
        let respawn_locs = consts::car::spawn::get_respawn_locations(self.config.game_mode);

        let mut kickoff_order_perm = ArrayVec::<usize, 5>::new();
        kickoff_order_perm.extend(0..kickoff_locs.len());
        if let Some(seed) = rng_seed {
            Rng::with_seed(seed).shuffle(&mut kickoff_order_perm);
        } else {
            self.rng.shuffle(&mut kickoff_order_perm);
        }

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
                    pos: Vec3A::new(spawn_pos.x, spawn_pos.y, consts::car::spawn::REST_Z),
                    rot_mat: Mat3A::IDENTITY,
                    vel: Vec3A::ZERO,
                    ang_vel: Vec3A::ZERO,
                },
                boost: self.config.mutators.car_spawn_boost_amount,
                is_on_ground: true,
                ..Default::default()
            };

            for cur_team in Team::ALL {
                let is_blue = cur_team == Team::Blue;

                let mut team_car_count = 0;
                let mut car_idx = None;

                for car in &self.cars {
                    if car.team == cur_team {
                        team_car_count += 1;

                        if team_car_count == i + 1 {
                            car_idx = Some(car.idx);
                            break;
                        }
                    }
                }

                if let Some(car_idx) = car_idx {
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
        }

        let mut ball_state = BallState::DEFAULT;
        match self.config.game_mode {
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
            GameMode::Dropshot => {
                self.update_tile_states();
            }
            _ => {}
        }

        self.set_ball_state(ball_state);

        if let Some(boost_pad_grid) = self.boost_pad_grid.as_mut() {
            boost_pad_grid.reset();
        }
    }

    /// Creates and adds a car to the arena, returning the index of the car in the cars vector
    pub fn add_car(&mut self, team: Team, config: CarBodyConfig) -> usize {
        let idx = self.cars.len();

        let mut car = Car::new(
            idx,
            team,
            &mut self.bullet_world,
            &self.config.mutators,
            config,
        );
        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.config.game_mode,
            self.config.mutators.car_spawn_boost_amount,
        );

        self.bullet_world.bodies_mut()[car.rigid_body_idx].user_pointer = idx;
        self.cars.push(car);
        idx
    }

    /// Steps the arena for 1 tick, returning the events produced during that tick
    pub fn step_tick(&mut self) -> &[ArenaEvent] {
        self.events.clear();

        // NOTE: This needs to be called manually
        // TODO: Make it not need to be called manually
        self.bullet_world.clear_accum_forces();

        // Limit velocities, then quantize physics values
        {
            use consts::{ball, car};

            for car_idx in 0..self.cars.len() {
                let car_rb = &mut self.bullet_world.bodies_mut()[self.cars[car_idx].rigid_body_idx];
                car_rb.limit_vels(car::MAX_SPEED * UU_TO_BT, car::MAX_ANG_SPEED);
                quantize::quantize(car_rb);
                // Sync the cached state after rigid-body limits.
                self.cars[car_idx].state.phys.vel = car_rb.lin_vel * BT_TO_UU;
                self.cars[car_idx].state.phys.ang_vel = car_rb.ang_vel;
            }
            let ball_rb = &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx];
            ball_rb.limit_vels(
                self.config.mutators.ball_max_speed * UU_TO_BT,
                ball::MAX_ANG_SPEED,
            );
            quantize::quantize(ball_rb);
            self.ball.state.phys.vel = ball_rb.lin_vel * BT_TO_UU;
            self.ball.state.phys.ang_vel = ball_rb.ang_vel;
        }

        // Keep resting balls active so same-tick contacts can affect them.

        for car in &mut self.cars {
            car.pre_tick_update(
                &mut self.bullet_world,
                &mut self.rng,
                self.config.game_mode,
                &self.config.mutators,
            );
        }

        self.ball.pre_tick_update(
            &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx],
            self.config.game_mode,
        );

        self.bullet_world
            .step_simulation(TICK_TIME, &mut self.contact_tracker);

        let contact_count = self.contact_tracker.num_records();
        for idx in 0..contact_count {
            let contact = *self.contact_tracker.get_record(idx);

            let bodies = self.bullet_world.bodies();
            let rb_a = &bodies[contact.rb_idx_a];
            let rb_b = &bodies[contact.rb_idx_b];
            let user_idx_a = rb_a.user_idx;
            let user_idx_b = rb_b.user_idx;
            let user_pointer_a = rb_a.user_pointer;
            let user_pointer_b = rb_b.user_pointer;

            match user_idx_a {
                UserInfoTypes::Car => match user_idx_b {
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
                },
                UserInfoTypes::Ball => match user_idx_b {
                    UserInfoTypes::DropshotTile => {
                        self.on_ball_tile_collision(user_pointer_b);
                    }
                    UserInfoTypes::None => {
                        self.on_ball_world_collision(&contact.manifold_point, contact.rb_idx_a);
                    }
                    _ => {}
                },
                _ => {}
            }
        }

        self.contact_tracker.clear_records();

        for car in &mut self.cars {
            car.post_tick_update(&mut self.bullet_world);
            let rb = &mut self.bullet_world.bodies_mut()[car.rigid_body_idx];
            car.finish_physics_tick(rb);

            if let Some(boost_pad_grid) = self.boost_pad_grid.as_mut() {
                let collected_pad_op = boost_pad_grid.maybe_give_car_boost(
                    &mut car.state,
                    &self.config.mutators,
                    self.tick_count,
                );

                if let Some(collected_pad_idx) = collected_pad_op {
                    self.events.push(CarPickupBoost(CarPickupBoostEvent {
                        car_idx: car.idx,
                        boost_pad_idx: collected_pad_idx,
                    }));
                }
            }
        }

        let ball_rb = &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx];
        self.ball.finish_physics_tick(ball_rb);

        if self.config.game_mode == GameMode::Dropshot
            && self.ball.state.ds_info.last_damage_tick == Some(self.tick_count)
        {
            self.update_tile_states();
        }

        self.tick_count += 1;

        if self.vis.is_some() {
            let arena_state = self.get_arena_state();
            if let Some(vis) = self.vis.as_mut() {
                vis.update(&arena_state, TICK_TIME);
            }
        };

        self.get_last_step_events()
    }

    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
    }

    #[inline]
    pub const fn game_mode(&self) -> GameMode {
        self.config.game_mode
    }

    #[inline]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.config.mutators
    }

    pub fn set_ball_state(&mut self, ball_state: BallState) {
        self.ball.set_state(
            &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx],
            ball_state,
        );
    }

    pub const fn get_ball_state(&self) -> &BallState {
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

    /// Debug access: per-wheel `(in_contact, suspension_length, suspension_rel_vel)`.
    pub fn get_car_wheel_debug(&self, car_idx: usize) -> [(bool, f32, f32); 4] {
        let mut out = [(false, 0.0f32, 0.0f32); 4];
        for (w, o) in self.cars[car_idx]
            .bullet_vehicle
            .wheels
            .iter()
            .zip(out.iter_mut())
        {
            if let Some(ri) = &w.raycast_info {
                *o = (true, ri.suspension_length, ri.suspension_relative_vel);
            }
        }
        out
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
        self.cars[car_idx].state.controls = controls;
    }

    pub fn respawn_car(&mut self, car_idx: usize) {
        let car = &mut self.cars[car_idx];

        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.config.game_mode,
            self.config.mutators.car_spawn_boost_amount,
        );
    }

    #[must_use]
    pub fn get_boost_pad_state(&self, idx: usize) -> BoostPadState {
        let pad = self.boost_pads()[idx];
        let cooldown = pad.gave_boost_tick_count.map_or(0.0, |gave_boost_tick| {
            let max_cooldown = pad.max_cooldown;
            let time_since = ((self.tick_count() as i64 - gave_boost_tick) as f32) * TICK_TIME;
            (max_cooldown - time_since).max(0.0)
        });

        BoostPadState { cooldown }
    }

    pub fn set_boost_pad_state(&mut self, idx: usize, state: BoostPadState) {
        let boost_pad_grid = self.boost_pad_grid.as_mut().unwrap();
        let tick_count = self.tick_count;
        let pad = &mut boost_pad_grid.all_pads[idx];
        if state.cooldown > 0.0 {
            let time_since_pickup = (pad.max_cooldown - state.cooldown).max(0.0);
            let ticks_since_pickup = (time_since_pickup * TICK_RATE).round() as i64;
            pad.gave_boost_tick_count = Some(tick_count as i64 - ticks_since_pickup);
        } else {
            boost_pad_grid.all_pads[idx].gave_boost_tick_count = None;
        }
    }

    #[must_use]
    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }

    pub(crate) fn boost_pads(&self) -> &[BoostPad] {
        &self.boost_pad_grid.as_ref().unwrap().all_pads
    }

    #[must_use]
    pub fn num_boost_pads(&self) -> usize {
        self.boost_pad_grid
            .as_ref()
            .map_or(0, |grid| grid.all_pads.len())
    }

    #[must_use]
    pub fn get_all_boost_pad_states(&self) -> Vec<BoostPadState> {
        (0..self.num_boost_pads())
            .map(|i| self.get_boost_pad_state(i))
            .collect()
    }

    #[must_use]
    pub fn get_all_boost_pad_configs(&self) -> Vec<BoostPadConfig> {
        (0..self.num_boost_pads())
            .map(|i| *self.get_boost_pad_config(i))
            .collect()
    }

    pub fn get_tile_states(&self) -> &TileStates {
        self.tile_states.as_ref().unwrap()
    }

    fn update_tile_states(&mut self) {
        for (team_idx, team_states) in self.tile_states.as_ref().unwrap().states.iter().enumerate()
        {
            for (tile_idx, &new_state) in team_states.iter().enumerate() {
                let rb_idx = tile_idx + consts::dropshot::NUM_TILES_PER_TEAM * team_idx;
                let tile_rb = &mut self.bullet_world.bodies_mut()[rb_idx];
                if new_state == TileDamageState::Broken {
                    tile_rb.collision_flags |= CollisionFlags::NoContactResponse;
                } else {
                    tile_rb.collision_flags &= !CollisionFlags::NoContactResponse;
                }
            }
        }
    }

    pub fn set_tile_states(&mut self, tile_states: TileStates) {
        self.tile_states = Some(tile_states);
        self.update_tile_states();
    }

    pub fn num_tiles(&self) -> usize {
        self.tile_states
            .as_ref()
            .map_or(0, |ts| ts.states[0].len() * ts.states.len())
    }

    #[must_use]
    pub fn get_arena_state(&self) -> ArenaState {
        let cars = self
            .cars
            .iter()
            .map(|c| (c.info, c.state))
            .collect::<Vec<_>>();
        let ball = *self.get_ball_state();

        let boost_pads = (0..self.num_boost_pads())
            .map(|i| (*self.get_boost_pad_config(i), self.get_boost_pad_state(i)))
            .collect();

        ArenaState {
            game_mode: self.config.game_mode,
            tick_count: self.tick_count,
            cars,
            ball,
            boost_pads,
            tile_states: self.tile_states,
        }
    }

    #[must_use]
    /// Returns the events generated during the last stepped tick
    pub fn get_last_step_events(&self) -> &[ArenaEvent] {
        self.events.events()
    }

    #[must_use]
    /// Cast N rays in the arena
    ///
    /// Note that rays are batch-casted 4 at a time for SIMD speed,
    /// so doing multiples of 4 at once is most efficient
    pub fn cast_rays(&self, ray_queries: &[RaycastQuery]) -> Vec<RaycastResult> {
        let mut results = Vec::with_capacity(ray_queries.len());
        for query_batch in ray_queries.chunks(4) {
            let (mut froms, mut tos) = ([Vec3A::ZERO; 4], [Vec3A::ZERO; 4]);
            for (i, query) in query_batch.iter().enumerate() {
                froms[i] = query.from;
                tos[i] = query.to;
            }

            let mut callback = ClosestQuadRayResultCallback::new(&froms, &tos, None);
            self.bullet_world.ray_test(&froms, &tos, &mut callback);

            for i in 0..query_batch.len() {
                let hit_info = if callback.has_hit(i) {
                    Some(RaycastHitInfo {
                        hit_point: callback.hit_point_world[i],
                        hit_normal: callback.hit_normal_world[i],
                        hit_fraction: callback.base.closest_hit_fraction[i],
                    })
                } else {
                    None
                };

                results.push(RaycastResult { hit_info })
            }
        }

        results
    }

    pub fn is_vis_enabled(&self) -> bool {
        self.vis.is_some()
    }
}

impl Arena {
    fn on_ball_tile_collision(&mut self, tile_idx: usize) {
        self.ball.on_dropshot_tile_collision(
            self.tile_states.as_mut().unwrap(),
            tile_idx,
            self.tick_count,
        );
    }

    fn on_ball_world_collision(&mut self, manifold_point: &ManifoldPoint, rb_index: usize) {
        let contact_point = manifold_point.pos_world_on_b * BT_TO_UU;
        let contact_normal = manifold_point.normal_world_on_b;

        let rb = &mut self.bullet_world.bodies_mut()[rb_index];
        self.ball
            .on_world_hit(rb, self.config.game_mode, contact_normal);

        self.events.push(BallHitWorld(BallHitWorldEvent {
            contact_point,
            contact_normal,
        }));
    }

    fn on_car_ball_collision(
        &mut self,
        car_idx: usize,
        manifold_point: &ManifoldPoint,
        ball_is_body_a: bool,
    ) {
        let ball_rb = &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx];
        let ball_lin_vel_before = ball_rb.lin_vel;
        self.ball.on_hit(
            &self.cars[car_idx],
            self.config.game_mode,
            &self.config.mutators,
            self.tick_count,
            ball_rb,
        );

        let contact_point = if ball_is_body_a {
            manifold_point.pos_world_on_a
        } else {
            manifold_point.pos_world_on_b
        } * BT_TO_UU;

        let extra_hit_vel = (ball_rb.lin_vel - ball_lin_vel_before) * BT_TO_UU;
        self.events.push(ArenaEvent::CarHitBall(CarHitBallEvent {
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
            let dir_to_victim = delta_pos.normalize_or_zero();

            let speed_towards_other_car = attacker_state.phys.vel.dot(dir_to_victim);
            let other_car_away_speed = victim_state.phys.vel.dot(vel_dir);
            if speed_towards_other_car <= other_car_away_speed {
                // Going towards other car slower than they're going away
                continue;
            }

            let local_point_x = if is_swapped {
                manifold_point.local_point_b
            } else {
                manifold_point.local_point_a
            }
            .x;

            let hit_with_bumper = local_point_x * BT_TO_UU > consts::car::bump::MIN_FORWARD_DIST;
            if !hit_with_bumper {
                // Didn't hit with bumper
                continue;
            }

            let mut is_demo = match self.config.mutators.demo_mode {
                DemoMode::OnContact => true,
                DemoMode::Disabled => false,
                DemoMode::Normal => attacker_state.is_supersonic,
            };
            if is_demo && !self.config.mutators.enable_team_demos {
                is_demo = attacker.team != victim.team;
            }

            if is_demo {
                victim.demolish(self.config.mutators.respawn_delay);
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
                    * self.config.mutators.bump_force_scale;
                let bump_impulse = (vel_dir * base_scale) + (hit_up_dir * upward_force);
                victim.vel_impulse_cache += bump_impulse * UU_TO_BT;
            }

            attacker.state.bump_cooldown_timer = self.config.mutators.bump_cooldown_time;

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

    #[cfg(debug_assertions)]
    pub fn get_car_impulse_history(
        &self,
        car_idx: usize,
    ) -> &IndexMap<(&'static str, bool), (Vec3A, Vec3A)> {
        let car_rb_index = self.cars[car_idx].rigid_body_idx;
        let rb = &self.bullet_world.bodies()[car_rb_index];
        &rb.dbg_tick_impulse_history
    }
}
