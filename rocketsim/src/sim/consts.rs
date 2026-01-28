use glam::Vec3A;

use crate::{GameMode, sim::linear_piece_curve::LinearPieceCurve};

pub struct PhysicsCoefs {
    pub friction: f32,
    pub restitution: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CarSpawnPos {
    pub x: f32,
    pub y: f32,
    pub yaw_ang: f32,
}

impl CarSpawnPos {
    #[inline]

    pub const fn new(x: f32, y: f32, yaw_ang: f32) -> Self {
        Self { x, y, yaw_ang }
    }
}

//////////////////////////

/// `BulletPhysics` Units (1m) to Unreal Units (2cm) conversion scale
pub(crate) const BT_TO_UU: f32 = 50.0;

/// Unreal Units (2cm) to `BulletPhysics` Units (1m) conversion scale
pub(crate) const UU_TO_BT: f32 = 1.0 / 50.0;

/// How many ticks occur in one second
pub const TICK_RATE: f32 = 120.0;

/// The amount of time each tick takes up (inverse of TICK_RATE)
pub const TICK_TIME: f32 = 1.0 / TICK_RATE;

/// The z-velocity added by gravity each second
pub const GRAVITY_Z: f32 = -650.0;

pub mod arena {
    use super::PhysicsCoefs;
    use crate::GameMode;
    use crate::shared::Aabb;
    use glam::Vec3A;

    pub const BASE_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.6,
        restitution: 0.3,
    };

    pub const fn get_aabb(game_mode: GameMode) -> Aabb {
        let (max_x, max_y, max_z) = match game_mode {
            GameMode::Hoops => (8900.0 / 3.0, 3581.0, 1820.0),
            GameMode::Dropshot => (5075.0, 4592.0, 2024.0),

            // Soccar arena
            _ => (4096.0, 5120.0, 2048.0),
        };

        let floor_height = match game_mode {
            GameMode::Dropshot => 1.5,
            _ => 0.0,
        };

        Aabb::new(
            Vec3A::new(-max_x, -max_y, floor_height),
            Vec3A::new(max_x, max_y, max_z),
        )
    }
}

pub mod car {
    use super::PhysicsCoefs;

    pub const MASS_BT: f32 = 180.0;

    pub const BASE_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.3,
        restitution: 0.1,
    };

    pub const HIT_BALL_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 2.0,
        restitution: 0.0,
    };

    pub const HIT_WORLD_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.3,
        restitution: 0.3,
    };

    pub const HIT_CAR_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.09,
        restitution: 0.1,
    };

    pub const MAX_SPEED: f32 = 2300.0;

    pub mod boost {
        pub const MAX: f32 = 100.0;
        pub const USED_PER_SECOND: f32 = MAX / 3.0;
        /// Minimum time we can be boosting for
        pub const MIN_TIME: f32 = 0.1;
        /// uu/s for vel (on the ground)
        pub const ACCEL_GROUND: f32 = 2975.0 / 3.0;
        /// uu/s for vel (airborne)
        pub const ACCEL_AIR: f32 = 3175.0 / 3.0;
        pub const SPAWN_AMOUNT: f32 = MAX / 3.0;
        /// Amount of boost recharged per second when recharging
        pub const RECHARGE_PER_SECOND: f32 = 10.0;
        /// Delay after the car stops boosting
        pub const RECHARGE_DELAY: f32 = 0.25;
    }

    /// Car can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 5.5;

    pub mod supersonic {
        pub const START_SPEED: f32 = 2200.0;
        pub const MAINTAIN_MIN_SPEED: f32 = START_SPEED - 100.0;
        pub const MAINTAIN_MAX_TIME: f32 = 1.0;
    }

    pub mod drive {
        pub const THROTTLE_TORQUE_AMOUNT: f32 = super::MASS_BT * 400.0;
        pub const BRAKE_TORQUE_AMOUNT: f32 = super::MASS_BT * (14.25 + (1.0 / 3.));
        /// If we are costing with less than this forward vel, we full-brake
        pub const STOPPING_FORWARD_VEL: f32 = 25.0;
        /// How much the brake is applied when coasting
        pub const COASTING_BRAKE_FACTOR: f32 = 0.15;
        /// If we are braking and moving faster than this, disable throttle
        pub const BRAKING_NO_THROTTLE_SPEED_THRESH: f32 = 0.01;
        /// Throttle input of less than this is ignored
        pub const THROTTLE_DEADZONE: f32 = 0.001;
        pub const THROTTLE_AIR_ACCEL: f32 = 200.0 / 3.0;

        pub const POWERSLIDE_RISE_RATE: f32 = 5.0;
        pub const POWERSLIDE_FALL_RATE: f32 = 2.0;
    }

    pub mod jump {
        pub const ACCEL: f32 = 4375.0 / 3.0;
        pub const IMMEDIATE_FORCE: f32 = 875.0 / 3.0;
        pub const MIN_TIME: f32 = 0.025;
        pub const RESET_TIME_PAD: f32 = 1.0 / 40.0;
        pub const MAX_TIME: f32 = 0.2;
        /// Can be at most 1.25 seconds after the jump is finished
        pub const DOUBLEJUMP_MAX_DELAY: f32 = 1.25;
    }

    pub mod flip {
        pub const Z_DAMP_120: f32 = 0.35;
        pub const Z_DAMP_START: f32 = 0.15;
        pub const Z_DAMP_END: f32 = 0.21;
        pub const TORQUE_TIME: f32 = 0.65;
        pub const TORQUE_MIN_TIME: f32 = 0.41;
        pub const PITCHLOCK_TIME: f32 = 1.0;
        pub const PITCHLOCK_EXTRA_TIME: f32 = 0.3;
        pub const INITIAL_VEL_SCALE: f32 = 500.0;
        /// Left/Right
        pub const TORQUE_X: f32 = 260.0;
        /// Forward/backward
        pub const TORQUE_Y: f32 = 224.0;
        pub const FORWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 1.0;
        pub const SIDE_IMPULSE_MAX_SPEED_SCALE: f32 = 1.9;
        pub const BACKWARD_IMPULSE_MAX_SPEED_SCALE: f32 = 2.5;
        pub const BACKWARD_IMPULSE_SCALE_X: f32 = 16.0 / 15.0;
    }

    pub mod air_control {
        use std::f32::consts::PI;

        use glam::Vec3A;

        pub const TORQUE: Vec3A = Vec3A::new(130., 95., 400.);
        pub const DAMPING: Vec3A = Vec3A::new(30., 20., 50.);
        pub const TORQUE_APPLY_SCALE: f32 = 2.0 * PI / (1 << 16) as f32 * 1000.0;
    }

    pub mod autoflip {
        use std::f32::consts::FRAC_1_SQRT_2;

        pub const IMPULSE: f32 = 200.0;
        pub const TORQUE: f32 = 50.0;
        pub const TIME: f32 = 0.4;
        pub const NORM_Z_THRESH: f32 = FRAC_1_SQRT_2;
        pub const ROLL_THRESH: f32 = 2.8;
    }

    pub mod autoroll {
        pub const FORCE: f32 = 100.0;
        pub const TORQUE: f32 = 80.0;
    }

    pub mod bump {
        pub const COOLDOWN_TIME: f32 = 0.25;
        pub const MIN_FORWARD_DIST: f32 = 64.5;
    }

    pub mod spawn {
        use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

        use crate::{GameMode, consts::CarSpawnPos};

        pub const REST_Z: f32 = 17.0;
        pub const SPAWN_Z: f32 = 36.0;

        pub const fn get_kickoff_spawn_locations(game_mode: GameMode) -> &'static [CarSpawnPos] {
            pub const LOCATIONS_SOCCAR: [CarSpawnPos; 5] = [
                CarSpawnPos::new(-2048., -2560., FRAC_PI_4 * 1.),
                CarSpawnPos::new(2048., -2560., FRAC_PI_4 * 3.),
                CarSpawnPos::new(-256., -3840., FRAC_PI_4 * 2.),
                CarSpawnPos::new(256., -3840., FRAC_PI_4 * 2.),
                CarSpawnPos::new(0., -4608., FRAC_PI_4 * 2.),
            ];
            pub const LOCATIONS_HOOPS: [CarSpawnPos; 5] = [
                CarSpawnPos::new(-1536., -3072., FRAC_PI_4 * 2.),
                CarSpawnPos::new(1536., -3072., FRAC_PI_4 * 2.),
                CarSpawnPos::new(-256., -2816., FRAC_PI_4 * 2.),
                CarSpawnPos::new(256., -2816., FRAC_PI_4 * 2.),
                CarSpawnPos::new(0., -3200., FRAC_PI_4 * 2.),
            ];
            pub const LOCATIONS_DROPSHOT: [CarSpawnPos; 5] = [
                CarSpawnPos::new(-1867., -2380., FRAC_PI_4 * 1.),
                CarSpawnPos::new(1867., -2380., FRAC_PI_4 * 3.),
                CarSpawnPos::new(-256., -3576., FRAC_PI_4 * 2.),
                CarSpawnPos::new(256., -3576., FRAC_PI_4 * 2.),
                CarSpawnPos::new(0., -4088., FRAC_PI_4 * 2.),
            ];
            pub const LOCATIONS_HEATSEEKER: [CarSpawnPos; 4] = [
                CarSpawnPos::new(-1000., -4620., FRAC_PI_2),
                CarSpawnPos::new(1000., -4620., FRAC_PI_2),
                CarSpawnPos::new(-2000., -4620., FRAC_PI_2),
                CarSpawnPos::new(2000., -4620., FRAC_PI_2),
            ];

            match game_mode {
                GameMode::Hoops => &LOCATIONS_HOOPS,
                GameMode::Dropshot => &LOCATIONS_DROPSHOT,
                GameMode::Heatseeker => &LOCATIONS_HEATSEEKER,
                _ => &LOCATIONS_SOCCAR,
            }
        }

        pub const RESPAWN_TIME: f32 = 3.0;

        pub const fn get_respawn_locations(game_mode: GameMode) -> &'static [CarSpawnPos] {
            const LOCATIONS_SOCCAR: [CarSpawnPos; 4] = [
                CarSpawnPos::new(-2304., -4608., FRAC_PI_2),
                CarSpawnPos::new(-2688., -4608., FRAC_PI_2),
                CarSpawnPos::new(2304., -4608., FRAC_PI_2),
                CarSpawnPos::new(2688., -4608., FRAC_PI_2),
            ];
            const LOCATIONS_HOOPS: [CarSpawnPos; 4] = [
                CarSpawnPos::new(-1920., -3072., FRAC_PI_2),
                CarSpawnPos::new(-1152., -3072., FRAC_PI_2),
                CarSpawnPos::new(1920., -3072., FRAC_PI_2),
                CarSpawnPos::new(1152., -3072., FRAC_PI_2),
            ];
            const LOCATIONS_DROPSHOT: [CarSpawnPos; 4] = [
                CarSpawnPos::new(-2176., -3410., FRAC_PI_2),
                CarSpawnPos::new(-1152., -3100., FRAC_PI_2),
                CarSpawnPos::new(2176., -3410., FRAC_PI_2),
                CarSpawnPos::new(1152., -3100., FRAC_PI_2),
            ];

            match game_mode {
                GameMode::Hoops => &LOCATIONS_HOOPS,
                GameMode::Dropshot => &LOCATIONS_DROPSHOT,
                _ => &LOCATIONS_SOCCAR,
            }
        }
    }
}

pub mod ball {
    use super::PhysicsCoefs;
    use crate::GameMode;

    pub const fn get_radius(game_mode: GameMode) -> f32 {
        pub const RADIUS_SOCCAR: f32 = 91.25;
        pub const RADIUS_HOOPS: f32 = 96.3831;
        pub const RADIUS_DROPSHOT: f32 = 100.2565;
        match game_mode {
            GameMode::Hoops => RADIUS_HOOPS,
            GameMode::Dropshot => RADIUS_DROPSHOT,
            GameMode::Snowday => f32::NAN,
            _ => RADIUS_SOCCAR,
        }
    }

    /// Ref: <https://www.reddit.com/r/RocketLeague/comments/bmje9l/comment/emxkwrl/?context=3>
    pub const MASS_BT: f32 = 30.0; // Ball mass divided by 6
    /// Greater than ball radius because of arena mesh collision margin
    pub const REST_Z: f32 = 93.15;
    /// Ball can never exceed this angular velocity (radians/s)
    pub const MAX_ANG_SPEED: f32 = 6.0;
    /// Net-velocity drag multiplier
    pub const DRAG: f32 = 0.03;
    pub const COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.35,
        restitution: 0.6,
    };

    /// Z impulse applied to hoops ball on kickoff
    pub const HOOPS_LAUNCH_Z_VEL: f32 = 1000.0;
    pub const HOOPS_LAUNCH_DELAY: f32 = 0.265;

    pub const MAX_SPEED: f32 = 6000.0;

    pub mod car_hit_impulse {
        pub const FORWARD_SCALE: f32 = 0.65;
        pub const MAX_DELTA_VEL_UU: f32 = 4600.0;

        pub const Z_SCALE_NORMAL: f32 = 0.35;
        pub const Z_SCALE_HOOPS_GROUND: f32 = Z_SCALE_NORMAL * 1.55;
        pub const Z_SCALE_HOOPS_NORMAL_Z_THRESH: f32 = 0.1;
    }
}

pub mod goal {
    use crate::Team;
    use crate::shared::Aabb;
    use glam::Vec3A;

    pub const SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y: f32 = 5124.25;
    pub const SOCCAR_GOAL_HEIGHT: f32 = 642.775; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/
    pub const SOCCAR_GOAL_HALF_WIDTH: f32 = 892.755; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/
    pub const SOCCAR_GOAL_DEPTH: f32 = 880.0; // https://wiki.rlbot.org/v4/botmaking/useful-game-values/

    pub const HOOPS_GOAL_SCORE_THRESHOLD_Z: f32 = 270.0;

    pub const fn get_goal_aabb(goal_team: Team) -> Aabb {
        const FRONT_Y: f32 = SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y;
        const DEPTH: f32 = SOCCAR_GOAL_DEPTH;
        let (min_y, max_y) = if goal_team.is_blue() {
            (-FRONT_Y - DEPTH, -FRONT_Y)
        } else {
            (FRONT_Y, FRONT_Y + DEPTH)
        };
        Aabb::new(
            Vec3A::new(-SOCCAR_GOAL_HALF_WIDTH, min_y, 0.0),
            Vec3A::new(SOCCAR_GOAL_HALF_WIDTH, max_y, SOCCAR_GOAL_HEIGHT),
        )
    }

    pub const fn get_goal_face_center(goal_team: Team) -> Vec3A {
        Vec3A::new(
            0.0,
            SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y * goal_team.get_y_dir(),
            SOCCAR_GOAL_HEIGHT / 2.0,
        )
    }
}

pub mod bullet_vehicle {
    pub const SUSPENSION_FORCE_SCALE_FRONT: f32 = 36.0 - (1.0 / 4.);
    pub const SUSPENSION_FORCE_SCALE_BACK: f32 = 54.0 + (1.0 / 4.) + (1.5 / 100.);
    pub const SUSPENSION_STIFFNESS: f32 = 500.0;
    pub const WHEELS_DAMPING_COMPRESSION: f32 = 25.0;
    pub const WHEELS_DAMPING_RELAXATION: f32 = 40.0;
    pub const MAX_SUSPENSION_TRAVEL: f32 = 12.0;
    pub const SUSPENSION_SUBTRACTION: f32 = 0.05;
}

pub mod curves {
    use super::LinearPieceCurve;

    pub const STEER_ANGLE_FROM_SPEED: LinearPieceCurve<6> = LinearPieceCurve::new([
        (0., 0.53356),
        (500., 0.31930),
        (1000., 0.18203),
        (1500., 0.10570),
        (1750., 0.08507),
        (3000., 0.03454),
    ]);
    pub const STEER_ANGLE_FROM_SPEED_THREEWHEEL: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.342_473), (2300., 0.034_837)]);
    pub const POWERSLIDE_STEER_ANGLE_FROM_SPEED: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.39235), (2500., 0.12610)]);
    pub const DRIVE_SPEED_TORQUE_FACTOR: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., 1.0), (1400., 0.1), (1410., 0.0)]);
    pub const NON_STICKY_FRICTION_FACTOR: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., 0.1), (0.7075, 0.5), (1., 1.0)]);
    pub const LAT_FRICTION: LinearPieceCurve<2> = LinearPieceCurve::new([(0., 1.0), (1., 0.2)]);
    pub const LAT_FRICTION_THREEWHEEL: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.30), (1., 0.25)]);
    pub const HANDBRAKE_LAT_FRICTION_FACTOR: f32 = 0.9;
    pub const HANDBRAKE_LONG_FRICTION_FACTOR: LinearPieceCurve<2> =
        LinearPieceCurve::new([(0., 0.5), (1., 0.9)]);
    pub const BALL_CAR_EXTRA_IMPULSE_FACTOR: LinearPieceCurve<4> =
        LinearPieceCurve::new([(0., 0.65), (500., 0.65), (2300., 0.55), (4600., 0.30)]);

    pub const BUMP_VEL_AMOUNT_GROUND: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., (5.0 / 6.)), (1400., 1100.), (2200., 1530.)]);
    pub const BUMP_VEL_AMOUNT_AIR: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., (5.0 / 6.)), (1400., 1390.), (2200., 1945.)]);
    pub const BUMP_UPWARD_VEL_AMOUNT: LinearPieceCurve<3> =
        LinearPieceCurve::new([(0., (2.0 / 6.)), (1400., 278.), (2200., 417.)]);
}

pub mod heatseeker {
    use std::f32::consts::PI;

    use glam::Vec3A;

    /// Initial target speed from kickoff (goes to 2985 after the first touch)
    pub const INITIAL_TARGET_SPEED: f32 = 2900.0;
    /// Increase of target speed each touch
    pub const TARGET_SPEED_INCREMENT: f32 = 85.0;
    /// Minimum time between touches to speed up
    pub const MIN_SPEEDUP_INTERVAL: f32 = 1.0;
    /// Y of target point in goal
    pub const TARGET_Y: f32 = 5120.0;
    /// Height of target point in goal
    pub const TARGET_Z: f32 = 320.0;
    /// Interpolation of horizontal (X+Y) turning
    pub const HORIZONTAL_BLEND: f32 = 1.45;
    /// Interpolation of vertical (Z) turning
    pub const VERTICAL_BLEND: f32 = 0.78;
    /// Interpolation of acceleration towards target speed
    pub const SPEED_BLEND: f32 = 0.3;
    /// Maximum pitch angle of turning
    pub const MAX_TURN_PITCH: f32 = 7000.0 * PI / (1 << 15) as f32;
    /// Maximum speed the ball can seek at (different from `BALL_MAX_SPEED`)
    pub const MAX_SPEED: f32 = 4600.0;
    /// Threshold of wall collision Y backwall distance to change goal targets
    pub const WALL_BOUNCE_CHANGE_Y_THRESH: f32 = 300.0;
    /// Threshold of Y normal to trigger bounce-back
    pub const WALL_BOUNCE_CHANGE_Y_NORMAL: f32 = 0.5;
    /// Scale of the extra wall bounce impulse
    pub const WALL_BOUNCE_FORCE_SCALE: f32 = 1.0 / 3.0;
    /// Fraction of upward bounce impulse that goes straight up
    pub const WALL_BOUNCE_UP_FRAC: f32 = 0.3;
    pub const BALL_START_POS: Vec3A = Vec3A::new(-1000., -2220., 92.75);
    pub const BALL_START_VEL: Vec3A = Vec3A::new(0., -65., 650.);
}

pub mod snowday {
    use super::*;

    /// Real puck radius varies a bit from point to point, but it shouldn't matter
    pub const PUCK_RADIUS: f32 = 114.25;
    pub const PUCK_HEIGHT: f32 = 62.5;
    /// Number of points on each circle of the cylinder
    pub const PUCK_CIRCLE_POINT_AMOUNT: f32 = 20.0;
    pub const PUCK_MASS_BT: f32 = 50.0;
    pub const PUCK_GROUND_STICK_FORCE: f32 = 70.0;
    pub const PUCK_COEFS: PhysicsCoefs = PhysicsCoefs {
        friction: 0.1,
        restitution: 0.3,
    };
}

pub mod dropshot {
    use super::{BT_TO_UU, Vec3A};

    pub const BALL_LAUNCH_Z_VEL: f32 = 985.0;
    pub const BALL_LAUNCH_DELAY: f32 = 0.26;
    /// Minimum downward speed to damage tiles
    pub const MIN_DOWNWARD_SPEED_TO_DAMAGE: f32 = 250.0;
    /// Minimum car->ball delta speed required to accumulate absorbed force
    pub const MIN_CHARGE_HIT_SPEED: f32 = 500.0;
    /// Minimum absorbed force to charge/"break open" the ball
    pub const MIN_ABSORBED_FORCE_FOR_CHARGE: f32 = 2500.0;
    /// Minimum absorbed force to super-charge the ball
    pub const MIN_ABSORBED_FORCE_FOR_SUPERCHARGE: f32 = 11000.0;
    /// Minimum time between damaging tiles
    pub const MIN_DAMAGE_INTERVAL: f32 = 0.1;
    pub const TILE_WIDTH_X: f32 = TILE_HEXAGON_VERTS_BT[1].to_array()[0] * 2.0 * BT_TO_UU;
    pub const ROW_OFFSET_Y: f32 = (TILE_HEXAGON_VERTS_BT[3].to_array()[1]
        + TILE_HEXAGON_VERTS_BT[4].to_array()[1])
        * BT_TO_UU;
    pub const TILE_OFFSET_Y: f32 = 2.54736 * BT_TO_UU;
    pub const NUM_TILES_PER_TEAM: i32 = 70;
    pub const TEAM_AMOUNT: i32 = 2;
    /// Number descends each row
    pub const TILES_IN_FIRST_ROW: i32 = 13;
    pub const TILES_IN_LAST_ROW: i32 = 7;
    pub const NUM_TILE_ROWS: i32 = TILES_IN_FIRST_ROW - TILES_IN_LAST_ROW + 1;
    pub const TILE_HEXAGON_AABB_MAX: Vec3A = Vec3A::new(7.6643, 8.85, 0.);
    pub const TILE_HEXAGON_VERTS_BT: [Vec3A; 6] = [
        Vec3A::new(0.0, -8.85, 0.),
        Vec3A::new(7.6643, -4.425, 0.),
        Vec3A::new(7.6643, 4.425, 0.),
        Vec3A::new(0.0, 8.85, 0.),
        Vec3A::new(-7.6643, 4.425, 0.),
        Vec3A::new(-7.6643, -4.425, 0.),
    ];
}

pub mod boost_pads {
    use super::{GameMode, Vec3A};

    // TODO: Do something about repetitive small/big pairs
    pub const CYL_HEIGHT: f32 = 95.0;
    pub const CYL_RAD_BIG: f32 = 208.0;
    pub const CYL_RAD_SMALL: f32 = 144.0;
    pub const BOX_HEIGHT: f32 = 64.0;
    pub const BOX_RAD_BIG: f32 = 160.0;
    pub const BOX_RAD_SMALL: f32 = 120.0;
    pub const COOLDOWN_BIG: f32 = 10.0;
    pub const COOLDOWN_SMALL: f32 = 4.0;
    pub const BOOST_AMOUNT_BIG: f32 = 100.0;
    pub const BOOST_AMOUNT_SMALL: f32 = 12.0;

    pub const fn get_locations(game_mode: GameMode, is_big: bool) -> &'static [Vec3A] {
        const LOCS_SMALL_SOCCAR: [Vec3A; 28] = [
            Vec3A::new(0., -4240., 70.),
            Vec3A::new(-1792., -4184., 70.),
            Vec3A::new(1792., -4184., 70.),
            Vec3A::new(-940., -3308., 70.),
            Vec3A::new(940., -3308., 70.),
            Vec3A::new(0., -2816., 70.),
            Vec3A::new(-3584., -2484., 70.),
            Vec3A::new(3584., -2484., 70.),
            Vec3A::new(-1788., -2300., 70.),
            Vec3A::new(1788., -2300., 70.),
            Vec3A::new(-2048., -1036., 70.),
            Vec3A::new(0., -1024., 70.),
            Vec3A::new(2048., -1036., 70.),
            Vec3A::new(-1024., 0., 70.),
            Vec3A::new(1024., 0., 70.),
            Vec3A::new(-2048., 1036., 70.),
            Vec3A::new(0., 1024., 70.),
            Vec3A::new(2048., 1036., 70.),
            Vec3A::new(-1788., 2300., 70.),
            Vec3A::new(1788., 2300., 70.),
            Vec3A::new(-3584., 2484., 70.),
            Vec3A::new(3584., 2484., 70.),
            Vec3A::new(0., 2816., 70.),
            Vec3A::new(-940., 3308., 70.),
            Vec3A::new(940., 3308., 70.),
            Vec3A::new(-1792., 4184., 70.),
            Vec3A::new(1792., 4184., 70.),
            Vec3A::new(0., 4240., 70.),
        ];
        const LOCS_BIG_SOCCAR: [Vec3A; 6] = [
            Vec3A::new(-3584., 0., 73.),
            Vec3A::new(3584., 0., 73.),
            Vec3A::new(-3072., 4096., 73.),
            Vec3A::new(3072., 4096., 73.),
            Vec3A::new(-3072., -4096., 73.),
            Vec3A::new(3072., -4096., 73.),
        ];

        const LOCS_SMALL_HOOPS: [Vec3A; 14] = [
            Vec3A::new(1536., -1024., 64.),
            Vec3A::new(-1280., -2304., 64.),
            Vec3A::new(0., -2816., 64.),
            Vec3A::new(-1536., -1024., 64.),
            Vec3A::new(1280., -2304., 64.),
            Vec3A::new(-512., 512., 64.),
            Vec3A::new(-1536., 1024., 64.),
            Vec3A::new(1536., 1024., 64.),
            Vec3A::new(1280., 2304., 64.),
            Vec3A::new(0., 2816., 64.),
            Vec3A::new(512., 512., 64.),
            Vec3A::new(512., -512., 64.),
            Vec3A::new(-512., -512., 64.),
            Vec3A::new(-1280., 2304., 64.),
        ];
        const LOCS_BIG_HOOPS: [Vec3A; 6] = [
            Vec3A::new(-2176., 2944., 72.),
            Vec3A::new(2176., -2944., 72.),
            Vec3A::new(-2176., -2944., 72.),
            Vec3A::new(-2432., 0., 72.),
            Vec3A::new(2432., 0., 72.),
            Vec3A::new(2175.99, 2944., 72.),
        ];

        match game_mode {
            GameMode::Hoops => {
                if is_big {
                    &LOCS_BIG_HOOPS
                } else {
                    &LOCS_SMALL_HOOPS
                }
            }
            _ => {
                if is_big {
                    &LOCS_BIG_SOCCAR
                } else {
                    &LOCS_SMALL_SOCCAR
                }
            }
        }
    }
}
