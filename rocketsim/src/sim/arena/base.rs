use std::{any::Any, f32::consts::PI, iter::repeat_n, mem};

use arrayvec::ArrayVec;
use fastrand::Rng;
use glam::{Affine3A, EulerRot, Mat3A, Vec3A};

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

/// Optional visualization hook, called with a snapshot after each tick.
///
/// Implement this in your own crate (see `rocketsim_vis`) to render or log
/// the game. Register with `arena.vis = Some(Box::new(my_vis))`.
pub trait Vis: Send + Sync + Any {
    /// Called after [`Arena::step_tick`] with the fresh state and `dt`
    /// ([`crate::consts::TICK_TIME`]).
    fn update(&mut self, arena_state: &ArenaState, dt: f32);
}

/// A full Rocket League game: ball, cars, boost pads/tiles, and physics.
///
/// Create with [`Arena::new`] (Soccar defaults) or
/// [`Arena::new_with_config`], then [`Arena::add_car`], then
/// [`Arena::reset_to_random_kickoff`]. Drive with
/// [`Arena::set_car_controls`] + [`Arena::step_tick`] at 120 Hz.
///
/// ```no_run
/// use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
/// init_from_default(true).unwrap();
/// let mut arena = Arena::new(GameMode::Soccar);
/// arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
/// arena.reset_to_random_kickoff(Some(0));
/// let events = arena.step_tick();
/// ```
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
    ball_only: bool,

    pub rng: Rng,
    pub vis: Option<Box<dyn Vis>>,
}

impl Arena {
    /// Creates a Soccar-defaults arena for `game_mode` (see [`ArenaConfig::new`]).
    ///
    /// # Panics
    ///
    /// Panics if collision meshes were not loaded via `crate::init*` (except
    /// [`GameMode::TheVoid`], which needs no meshes).
    #[must_use]
    pub fn new(game_mode: GameMode) -> Self {
        Self::new_with_config(ArenaConfig::new(game_mode))
    }

    /// Creates an arena from a full [`ArenaConfig`] (mutators, pads, RNG, ...).
    ///
    /// # Panics
    ///
    /// Same as [`Arena::new`]: panics without loaded meshes (except `TheVoid`).
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
        let ball_only = tile_states.is_none();

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
            ball_only,

            vis: None,
        }
    }

    /// Borrow the config this arena was built with.
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

    /// Returns `true` if the ball is fully inside a goal (Soccar/Hoops) or
    /// has fallen through broken Dropshot tiles. Always `false` for `TheVoid`.
    ///
    /// Check this after [`Arena::step_tick`]; RocketSim does not reset or
    /// award points itself.
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

    /// Teleports cars to kickoff spawns and resets ball/pads (Dropshot tiles too).
    ///
    /// Cars are assigned in team order to shuffled
    /// `consts::car::spawn::get_kickoff_spawn_locations`. Orange spawns are
    /// mirrored/rotated 180°. Extra cars past the 5 kickoff spots overflow to
    /// respawn locations. `rng_seed` overrides the arena RNG for this call
    /// only; `None` uses the arena RNG (seed via `ArenaConfig::with_rng_seed`
    /// for replays).
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
                self.tile_states = Some(TileStates::DEFAULT);
                self.ball_only = false;
                self.update_tile_states();
            }
            _ => {}
        }

        self.set_ball_state(ball_state);

        if let Some(boost_pad_grid) = self.boost_pad_grid.as_mut() {
            boost_pad_grid.reset();
        }
    }

    /// Spawns a car, respawns it at a team respawn pad, and returns its index.
    ///
    /// The index is the car's id for `get_car_*/set_car_*`. Cars can only be
    /// added, never removed — build a fresh [`Arena`] to reset the lobby.
    ///
    /// ```no_run
    /// # use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};
    /// # init_from_default(true).unwrap();
    /// # let mut arena = Arena::new(GameMode::Soccar);
    /// let blue = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    /// let orange = arena.add_car(Team::Orange, CarBodyConfig::DOMINUS);
    /// ```
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
        self.ball_only = false;
        idx
    }

    /// Advances the game one tick (`1/120` s) and returns that tick's events.
    ///
    /// Set inputs with [`Arena::set_car_controls`] before calling. The
    /// returned slice is only valid until the next `step_tick` — copy it if
    /// you need it longer. See [`Arena::get_last_step_events`].
    ///
    /// ```no_run
    /// # use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default};
    /// # init_from_default(true).unwrap();
    /// # let mut arena = Arena::new(GameMode::Soccar);
    /// # let car = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);
    /// arena.set_car_controls(car, CarControls::default().with_throttle(1.0));
    /// for event in arena.step_tick() {
    ///     println!("{event:?}");
    /// }
    /// ```
    pub fn step_tick(&mut self) -> &[ArenaEvent] {
        self.events.clear();

        // NOTE: This needs to be called manually
        // TODO: Make it not need to be called manually
        self.bullet_world.clear_accum_forces();

        // Limit velocities, then quantize physics values
        {
            use consts::{ball, car};

            if !self.ball_only {
                for car_idx in 0..self.cars.len() {
                    let car_rb =
                        &mut self.bullet_world.bodies_mut()[self.cars[car_idx].rigid_body_idx];
                    car_rb.limit_vels(car::MAX_SPEED * UU_TO_BT, car::MAX_ANG_SPEED);
                    quantize::quantize(car_rb);
                    // Sync the cached state after rigid-body limits.
                    self.cars[car_idx].state.phys.vel = car_rb.lin_vel * BT_TO_UU;
                    self.cars[car_idx].state.phys.ang_vel = car_rb.ang_vel;
                }
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

        if !self.ball_only {
            for car in &mut self.cars {
                car.pre_tick_update(
                    &mut self.bullet_world,
                    &mut self.rng,
                    self.config.game_mode,
                    &self.config.mutators,
                );
            }
        }

        self.ball.pre_tick_update(
            &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx],
            self.config.game_mode,
        );

        let ball_only = self.ball_only;
        self.contact_tracker.set_ball_only(ball_only);
        self.bullet_world.set_ball_only(ball_only);

        self.bullet_world
            .step_simulation(TICK_TIME, &mut self.contact_tracker);

        if ball_only {
            let contact_count = self.contact_tracker.num_ball_world_records();
            for idx in 0..contact_count {
                let contact = *self.contact_tracker.get_ball_world_record(idx);
                self.on_ball_world_collision_parts(
                    contact.rb_idx,
                    contact.contact_point,
                    contact.contact_normal,
                );
            }
        } else {
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
        }

        self.contact_tracker.clear_records();

        if !self.ball_only {
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

    /// Ticks simulated so far (starts at 0, increments per [`Arena::step_tick`]).
    #[inline]
    pub const fn tick_count(&self) -> u64 {
        self.tick_count
    }

    /// The game mode this arena was built with.
    #[inline]
    pub const fn game_mode(&self) -> GameMode {
        self.config.game_mode
    }

    /// Active mutators (physics/rules). Set at construction via `ArenaConfig`.
    #[inline]
    pub const fn mutator_config(&self) -> &MutatorConfig {
        &self.config.mutators
    }

    /// Teleports the ball (position/velocity/rotation + mode state).
    ///
    /// Use to place kickoffs, restore snapshots, or inject test states.
    /// For Heatseeker also set `hs_info`; for Dropshot `ds_info`.
    pub fn set_ball_state(&mut self, ball_state: BallState) {
        self.ball.set_state(
            &mut self.bullet_world.bodies_mut()[self.ball.rigid_body_idx],
            ball_state,
        );
    }

    /// Current ball state (updated after every [`Arena::step_tick`]).
    pub const fn get_ball_state(&self) -> &BallState {
        &self.ball.state
    }

    /// All cars in add order; index `i` is the id from [`Arena::add_car`].
    #[inline]
    pub const fn cars(&self) -> &Vec<Car> {
        &self.cars
    }

    /// Number of cars added with [`Arena::add_car`].
    #[inline]
    pub const fn num_cars(&self) -> usize {
        self.cars.len()
    }

    /// Immutable car info (team, hitbox preset). Panics if `car_idx` is OOB.
    pub fn get_car_info(&self, car_idx: usize) -> &CarInfo {
        &self.cars[car_idx].info
    }

    /// Current car state (updated after every [`Arena::step_tick`]).
    pub fn get_car_state(&self, car_idx: usize) -> &CarState {
        self.cars[car_idx].get_state()
    }

    /// Last controls set for a car (equals applied controls after the tick).
    pub fn get_car_controls(&self, car_idx: usize) -> &CarControls {
        &self.cars[car_idx].state.controls
    }

    /// Re-syncs the sticky wheel-contact gate after externally teleporting a car.
    ///
    /// Call after [`Arena::set_car_state`] when you restore snapshots mid-drive,
    /// otherwise the next tick may apply a one-tick sticking artifact.
    /// Normal `set_car_controls` + `step_tick` loops don't need this.
    pub fn refresh_car_sticky_gate(&mut self, car_idx: usize) {
        self.cars[car_idx].refresh_sticky_gate(&self.bullet_world);
    }

    /// Convenience getter returning `(info, state)` together.
    pub fn get_car_info_and_state(&self, car_idx: usize) -> (&CarInfo, &CarState) {
        let car = &self.cars[car_idx];
        (&car.info, &car.state)
    }

    /// Teleports a car (position/velocity/boost/...). Clears cached impulses.
    ///
    /// If you restore mid-drive snapshots every tick (replay/RLBot), follow
    /// with [`Arena::refresh_car_sticky_gate`].
    pub fn set_car_state(&mut self, car_idx: usize, state: CarState) {
        let car = &mut self.cars[car_idx];

        car.set_state(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &state,
        );
    }

    /// Sets the inputs applied on the next [`Arena::step_tick`].
    ///
    /// Analog inputs are clamped to `-1..1` (see [`CarControls::clamp`]).
    /// `jump` is edge-triggered (only the rising edge jumps/flips).
    pub fn set_car_controls(&mut self, car_idx: usize, controls: CarControls) {
        self.cars[car_idx].state.controls = controls;
    }

    /// Respawns a car at a random team respawn pad with spawn boost.
    ///
    /// Normally demos auto-respawn after `mutators.respawn_delay`; use this
    /// to force it (e.g. after [`Arena::reset_to_random_kickoff`] tests).
    pub fn respawn_car(&mut self, car_idx: usize) {
        let car = &mut self.cars[car_idx];

        car.respawn(
            &mut self.bullet_world.bodies_mut()[car.rigid_body_idx],
            &mut self.rng,
            self.config.game_mode,
            self.config.mutators.car_spawn_boost_amount,
        );
    }

    /// Remaining cooldown for a pad in seconds (`0` = active).
    ///
    /// # Panics
    ///
    /// Panics in `TheVoid`/`Dropshot` (no pads) or for an OOB index.
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

    /// Forces a pad cooldown (e.g. restore a snapshot). `cooldown <= 0`
    /// reactivates the pad.
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

    /// Static pad layout (`pos`, `is_big`). Order is RLBot/RLGym (Y, then X).
    #[must_use]
    pub fn get_boost_pad_config(&self, idx: usize) -> &BoostPadConfig {
        self.boost_pads()[idx].config()
    }

    pub(crate) fn boost_pads(&self) -> &[BoostPad] {
        &self.boost_pad_grid.as_ref().unwrap().all_pads
    }

    /// Number of pads (`0` in `TheVoid`/`Dropshot`; 34 Soccar, 20 Hoops).
    #[must_use]
    pub fn num_boost_pads(&self) -> usize {
        self.boost_pad_grid
            .as_ref()
            .map_or(0, |grid| grid.all_pads.len())
    }

    /// All pad cooldowns in pad order (snapshot helper).
    #[must_use]
    pub fn get_all_boost_pad_states(&self) -> Vec<BoostPadState> {
        (0..self.num_boost_pads())
            .map(|i| self.get_boost_pad_state(i))
            .collect()
    }

    /// All pad layouts in pad order (snapshot helper).
    #[must_use]
    pub fn get_all_boost_pad_configs(&self) -> Vec<BoostPadConfig> {
        (0..self.num_boost_pads())
            .map(|i| *self.get_boost_pad_config(i))
            .collect()
    }

    /// Dropshot tile damage grid. Panics outside Dropshot.
    pub fn get_tile_states(&self) -> &TileStates {
        self.tile_states.as_ref().unwrap()
    }

    fn update_tile_states(&mut self) {
        let tile_states = self.tile_states.as_ref().unwrap().states;
        for rb in self.bullet_world.bodies_mut() {
            if rb.user_idx != UserInfoTypes::DropshotTile {
                continue;
            }
            let team_idx = rb.user_pointer / consts::dropshot::NUM_TILES_PER_TEAM;
            let tile_idx = rb.user_pointer % consts::dropshot::NUM_TILES_PER_TEAM;
            if tile_states[team_idx][tile_idx] == TileDamageState::Broken {
                rb.collision_flags |= CollisionFlags::NoContactResponse;
            } else {
                rb.collision_flags &= !CollisionFlags::NoContactResponse;
            }
        }
    }

    /// Replaces the Dropshot tile grid and updates collision (broken tiles
    /// stop colliding). Also clears ball-only mode so tiles simulate.
    pub fn set_tile_states(&mut self, tile_states: TileStates) {
        self.tile_states = Some(tile_states);
        self.ball_only = false;
        self.update_tile_states();
    }

    /// Total tiles across both teams (`0` outside Dropshot, else 140).
    pub fn num_tiles(&self) -> usize {
        self.tile_states
            .as_ref()
            .map_or(0, |ts| ts.states[0].len() * ts.states.len())
    }

    /// Full snapshot (cars, ball, pads, tiles) for replays/debugging.
    ///
    /// Cheap clones of state only (no physics-world copy). Pair with
    /// `set_*` methods to restore. [`crate::ArenaState::new_empty`] builds
    /// an empty placeholder when you only need the type.
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

    /// Events from the last [`Arena::step_tick`] (same slice `step_tick` returned).
    ///
    /// Cleared at the start of each tick. See [`ArenaEvent`] for variants.
    #[must_use]
    pub fn get_last_step_events(&self) -> &[ArenaEvent] {
        self.events.events()
    }

    /// Casts rays against arena + cars + ball, 4 at a time via SIMD.
    ///
    /// Batch multiples of 4 for best throughput. Each ray must be shorter
    /// than the broadphase cell size (`ArenaConfig::max_aabb_len`, 370 uu by
    /// default) — longer rays panic in debug. Set
    /// [`RaycastQuery::hit_dynamic`] per query (currently informational —
    /// static + dynamic are both tested).
    ///
    /// # Units caveat
    ///
    /// Queries are currently passed to the Bullet world **unconverted**, so
    /// they must be in Bullet units (`uu * UU_TO_BT`, i.e. divide by 50) and
    /// hits come back in Bullet units too (multiply by `BT_TO_UU`). This
    /// disagrees with the rest of the API (Unreal units) and is almost
    /// certainly a bug — TODO: convert inside `cast_rays`. Until then:
    ///
    /// ```no_run
    /// # use rocketsim::{Arena, GameMode, RaycastQuery, init_from_default};
    /// # use rocketsim::consts::{BT_TO_UU, UU_TO_BT};
    /// # init_from_default(true).unwrap();
    /// # let arena = Arena::new(GameMode::Soccar);
    /// # let from_uu = arena.get_ball_state().pos;
    /// # let to_uu = from_uu - glam::Vec3A::new(0., 0., 300.);
    /// let results = arena.cast_rays(&[RaycastQuery {
    ///     from: from_uu * UU_TO_BT,
    ///     to: to_uu * UU_TO_BT,
    ///     hit_dynamic: false,
    /// }]);
    /// if let Some(hit) = results[0].hit_info {
    ///     let point_uu = hit.hit_point * BT_TO_UU;
    ///     assert!(point_uu.z >= 0.0);
    /// }
    /// ```
    #[must_use]
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

    /// Returns `true` when a [`Vis`] hook is registered.
    pub fn is_vis_enabled(&self) -> bool {
        self.vis.is_some()
    }
}

fn is_within_rect_cone(
    forward: Vec3A,
    right: Vec3A,
    up: Vec3A,
    delta: Vec3A,
    b_reverse: bool,
    yaw_limit_deg: f32,
    pitch_limit_deg: f32,
) -> bool {
    debug_assert!(yaw_limit_deg < 90.0 && pitch_limit_deg < 90.0);

    let fwd = if b_reverse { -forward } else { forward };
    let Some(d) = delta.try_normalize() else {
        return false;
    };

    let f = d.dot(fwd);
    if f <= 0.0 {
        return false;
    }

    // angle = atan2(|component|, f) <= limit  <=>  |component| <= f * tan(limit)
    d.dot(right).abs() <= f * yaw_limit_deg.to_radians().tan()
        && d.dot(up).abs() <= f * pitch_limit_deg.to_radians().tan()
}

fn is_within_demo_cone(
    forward: Vec3A,
    right: Vec3A,
    up: Vec3A,
    delta: Vec3A,
    b_reverse: bool,
) -> bool {
    use consts::car::demo;

    is_within_rect_cone(
        forward,
        right,
        up,
        delta,
        b_reverse,
        demo::YAW_LIMIT_DEG,
        demo::PITCH_LIMIT_DEG,
    )
}

fn is_within_bump_cone(
    forward: Vec3A,
    right: Vec3A,
    up: Vec3A,
    delta: Vec3A,
    b_reverse: bool,
) -> bool {
    use consts::car::bump;

    is_within_rect_cone(
        forward,
        right,
        up,
        delta,
        b_reverse,
        bump::YAW_LIMIT_DEG,
        bump::PITCH_LIMIT_DEG,
    )
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
        self.on_ball_world_collision_parts(
            rb_index,
            manifold_point.pos_world_on_b,
            manifold_point.normal_world_on_b,
        );
    }

    fn on_ball_world_collision_parts(
        &mut self,
        rb_index: usize,
        contact_point: Vec3A,
        contact_normal: Vec3A,
    ) {
        let contact_point = contact_point * BT_TO_UU;

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
            &mut self.cars[car_idx],
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

            if !is_within_bump_cone(
                attacker_state.phys.get_forward_dir(),
                attacker_state.phys.get_right_dir(),
                attacker_state.phys.get_up_dir(),
                delta_pos,
                false,
            ) {
                continue;
            }

            let mut is_demo = match self.config.mutators.demo_mode {
                DemoMode::OnContact => true,
                DemoMode::Disabled => false,
                DemoMode::Normal => {
                    attacker_state.is_supersonic && {
                        let fwd_speed = attacker_state
                            .phys
                            .vel
                            .dot(attacker_state.phys.get_forward_dir());
                        fwd_speed.abs() >= consts::car::supersonic::MAINTAIN_MIN_SPEED
                            && is_within_demo_cone(
                                attacker_state.phys.get_forward_dir(),
                                attacker_state.phys.get_right_dir(),
                                attacker_state.phys.get_up_dir(),
                                delta_pos,
                                fwd_speed < 0.0,
                            )
                    }
                }
            };
            if is_demo && !self.config.mutators.enable_team_demos {
                is_demo = attacker.team != victim.team;
            }

            if is_demo {
                victim.demolish(self.config.mutators.respawn_delay);
            } else {
                let ground_hit = victim_state.is_on_ground;
                let attacker_speed = attacker_state.phys.vel.length().min(2200.0);
                if ground_hit {
                    // Grounded victim: Ground and Z curves on total attacker speed.
                    let base_scale =
                        consts::curves::BUMP_VEL_AMOUNT_GROUND.get_output(attacker_speed);

                    let hit_up_dir = victim_state.phys.rot_mat.z_axis;

                    let upward_vel_curve = &consts::curves::BUMP_UPWARD_VEL_AMOUNT;
                    let upward_force = upward_vel_curve.get_output(attacker_speed)
                        * self.config.mutators.bump_force_scale;
                    let bump_impulse = (vel_dir * base_scale) + (hit_up_dir * upward_force);
                    victim.vel_impulse_cache += bump_impulse * UU_TO_BT;
                } else {
                    // Airborne victim: Air curve on total attacker speed; no up term.
                    let base_scale = consts::curves::BUMP_VEL_AMOUNT_AIR.get_output(attacker_speed);
                    let bump_impulse = vel_dir * base_scale;
                    victim.vel_impulse_cache += bump_impulse * UU_TO_BT;
                }
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
}
