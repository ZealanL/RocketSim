#pragma once
#include "BaseInc.h"
#include "Math/Math.h"

// Constant/default values from the game

namespace RLConst {
	constexpr float GRAVITY_Z = -650.f;

	constexpr float
		ARENA_EXTENT_X = 4096,
		ARENA_EXTENT_Y = 5120, // Does not include inner-goal
		ARENA_HEIGHT = 2048;

	constexpr float CAR_MASS_BT = 180.f;
	constexpr float BALL_MASS_BT = CAR_MASS_BT / 6.f; // Ref: https://www.reddit.com/r/RocketLeague/comments/bmje9l/comment/emxkwrl/?context=3

	constexpr float CAR_COLLISION_FRICTION = 0.3f;
	constexpr float CAR_COLLISION_RESTITUTION = 0.1f;

	constexpr float CARBALL_COLLISION_FRICTION = 2.0f;
	constexpr float CARBALL_COLLISION_RESTITUTION = 0.0f;

	constexpr float CARWORLD_COLLISION_FRICTION = 0.3f;
	constexpr float CARWORLD_COLLISION_RESTITUTION = 0.3f;

	constexpr float CARCAR_COLLISION_FRICTION = 0.09f;
	constexpr float CARCAR_COLLISION_RESTITUTION = 0.1f;

	constexpr float BALL_MAX_ANG_SPEED = 6.f; // Ball can never exceed this angular velocity (radians/s)
	constexpr float BALL_DRAG = 0.03f; // Net-velocity drag multiplier
	constexpr float BALL_FRICTION = 0.35f;
	constexpr float BALL_RESTITUTION = 0.6f; // Bounce factor

	constexpr float CAR_MAX_SPEED = 2300.f;
	constexpr float BALL_MAX_SPEED = 6000.f;

	constexpr float BOOST_MAX = 100.f;
	constexpr float BOOST_USED_PER_SECOND = BOOST_MAX / 3;
	constexpr float BOOST_MIN_TIME = 0.1f; // Minimum time we can be boosting for
	constexpr float BOOST_FORCE = 3816.f;
	constexpr float BOOST_SPAWN_AMOUNT = BOOST_MAX / 3;

	constexpr float
		BOOST_FORCE_GROUND_DECAY_MIN_VEL = 600.f,
		BOOST_FORCE_GROUND_DECAY_AMOUNT = 0.072f;

	constexpr float CAR_MAX_ANG_SPEED = 5.5f; // Car can never exceed this angular velocity (radians/s)

	// Speed needed to begin being supersonic
	constexpr float SUPERSONIC_START_SPEED = 2200.f; 
	
	// We can still be technically supersonic at this speed if we slow down after reaching supersonic, 
	// until the timer runs out (see timer's time below)
	constexpr float SUPERSONIC_MAINTAIN_MIN_SPEED = SUPERSONIC_START_SPEED - 100.f;

	// How long we can maintain supersonic status while (speed >= SUPERSONIC_MAINTAIN_MIN_SPEED and < SUPERSONIC_START_SPEED)
	constexpr float SUPERSONIC_MAINTAIN_MAX_TIME = 1.f; 

	// Powerslide is actually an analog value from 0-1
	// These values are are in powerslide-per-second rates for rise and fall
	// Thanks to Rangler for this one! (I did later stumble upon it myself but Rangler already knew about it)
	constexpr float 
		POWERSLIDE_RISE_RATE = 5, 
		POWERSLIDE_FALL_RATE = 2;

	constexpr float
		THROTTLE_TORQUE_AMOUNT = CAR_MASS_BT * 400.f,
		BRAKE_TORQUE_AMOUNT = CAR_MASS_BT * (14.25f + (1.f/3.f));
		
	constexpr float
		STOPPING_FORWARD_VEL = 25.f, // If we are costing with less than this forward vel, we full-brake
		COASTING_BRAKE_FACTOR = 0.15f, // How much the brake is applied when costing
		THROTTLE_DEADZONE = 0.001f, // Throttle input of less than this is ignored
		THROTTLE_AIR_FORCE = CAR_MASS_BT * (1 / 0.75f);

	constexpr float
		JUMP_ACCEL = 4375.f / 3.f,
		JUMP_IMMEDIATE_FORCE = 875.f / 3.f,
		JUMP_MIN_TIME = 0.025f,
		JUMP_RESET_TIME_PAD = (1 / 40.f),
		JUMP_MAX_TIME = 0.2f,
		DOUBLEJUMP_MAX_DELAY = 1.25f; // Can be at most 1.25 seconds after the jump is finished

	// Mostly from: https://github.com/samuelpmish/RLUtilities/blob/develop/src/mechanics/dodge.cc
	constexpr float
		FLIP_Z_DAMP_120 = 0.35f,
		FLIP_Z_DAMP_START = 0.15f,
		FLIP_Z_DAMP_END = 0.21f,
		FLIP_TORQUE_TIME = 0.65f,
		FLIP_TORQUE_MIN_TIME = 0.41f,
		FLIP_PITCHLOCK_TIME = 1.f,
		FLIP_INITIAL_VEL_SCALE = 500.f,
		FLIP_TORQUE_X = 260.f, // Left/Right
		FLIP_TORQUE_Y = 224.f, // Forward/backward
		FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE = 1.f,
		FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE = 1.9f,
		FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE = 2.5f,
		FLIP_BACKWARD_IMPULSE_SCALE_X = 16.f / 15.f;

	// Rocket League uses BulletPhysics, so I'd imagine they use a variation of the btRaycastVehicle
	// These are those vehicle's settings
	namespace BTVehicle {
		// TODO: These values might change from car to car...? Need to check!
		constexpr float 
			SUSPENSION_FORCE_SCALE_FRONT = 36.f - (1.f / 4.f),
			SUSPENSION_FORCE_SCALE_BACK = 54.f + (1.f / 4.f) + (1.5f / 100.f);

		constexpr float
			SUSPENSION_STIFFNESS = 500.f,
			WHEELS_DAMPING_COMPRESSION = 25.f,
			WHEELS_DAMPING_RELAXATION = 40.f,
			MAX_SUSPENSION_TRAVEL = 12.f; // TODO: Are we sure this is the same for all cars?
	}

	// NOTE: Angle order is PYR
	const static Vec
		CAR_AIR_CONTROL_TORQUE = Vec(130, 95, 400),
		CAR_AIR_CONTROL_DAMPING = Vec(30, 20, 50);

	// TODO: Dropshot one may be a little off, will check at some future point
	constexpr float
		BALL_COLLISION_RADIUS_NORMAL = 91.25f, // Soccar, Hoops, etc.
		BALL_COLLISION_RADIUS_DROPSHOT = 103.6f;

	constexpr float SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y = 5121.75f;
	constexpr float SOCCAR_BALL_SCORE_THRESHOLD_Y = SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y + BALL_COLLISION_RADIUS_NORMAL;

	constexpr float CAR_TORQUE_SCALE = 0.09587379924f;

	constexpr float
		CAR_AUTOFLIP_IMPULSE = 200,
		CAR_AUTOFLIP_TORQUE = 1000,
		CAR_AUTOFLIP_TIME = 0.4f,
		CAR_AUTOFLIP_NORMZ_THRESH = M_SQRT1_2;

	constexpr float
		CAR_AUTOROLL_FORCE = 100,
		CAR_AUTOROLL_TORQUE = 80;

	constexpr float
		BALL_CAR_EXTRA_IMPULSE_Z_SCALE = 0.35f,
		BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE = 0.65f,
		BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU = 4600.f;

	namespace BoostPads {
		// Mostly from a Rocket Science video: https://www.youtube.com/watch?v=xgfa-qZyInw
		
		constexpr float
			CYL_HEIGHT = 95,
			CYL_RAD_BIG = 208,
			CYL_RAD_SMALL = 144;

		constexpr float
			BOX_HEIGHT = 64,
			BOX_RAD_BIG = 160,
			BOX_RAD_SMALL = 120;

		constexpr float
			COOLDOWN_BIG = 10,
			COOLDOWN_SMALL = 4;

		constexpr float
			BOOST_AMOUNT_BIG = 100,
			BOOST_AMOUNT_SMALL = 12;

		constexpr int
			LOCS_AMOUNT_SMALL = 28,
			LOCS_AMOUNT_BIG = 6;

		const static Vec LOCS_SMALL[LOCS_AMOUNT_SMALL] = {
			{0.f,		-4240.f,	70.f },
			{-1792.f,	-4184.f,	70.f },
			{1792.f,	-4184.f,	70.f },
			{-940.f,	-3308.f,	70.f },
			{940.f,		-3308.f,	70.f },
			{0.f,		-2816.f,	70.f },
			{-3584.f,	-2484.f,	70.f },
			{3584.f,	-2484.f,	70.f },
			{-1788.f,	-2300.f,	70.f },
			{1788.f,	-2300.f,	70.f },
			{-2048.f,	-1036.f,	70.f },
			{0.f,		-1024.f,	70.f },
			{2048.f,	-1036.f,	70.f },
			{-1024.f,	0.f,		70.f },
			{1024.f,	0.f,		70.f },
			{-2048.f,	1036.f,		70.f },
			{0.f,		1024.f,		70.f },
			{2048.f,	1036.f,		70.f },
			{-1788.f,	2300.f,		70.f },
			{1788.f,	2300.f,		70.f },
			{-3584.f,	2484.f,		70.f },
			{3584.f,	2484.f,		70.f },
			{0.f,		2816.f,		70.f },
			{-940.f,	3310.f,		70.f },
			{940.f,		3308.f,		70.f },
			{-1792.f,	4184.f,		70.f },
			{1792.f,	4184.f,		70.f },
			{0.f,		4240.f,		70.f }
		};

		const static Vec LOCS_BIG[LOCS_AMOUNT_BIG] = {
			{-3584.f,     0.f, 73.f },
			{ 3584.f,     0.f, 73.f },
			{-3072.f,  4096.f, 73.f },
			{ 3072.f,  4096.f, 73.f },
			{-3072.f, -4096.f, 73.f },
			{ 3072.f, -4096.f, 73.f }
		};
	}

	constexpr float CAR_SPAWN_REST_Z = 17.f;
	constexpr float CAR_RESPAWN_Z = 36.f;

	constexpr int CAR_SPAWN_LOCATION_AMOUNT = 5;
	constexpr int CAR_RESPAWN_LOCATION_AMOUNT = 4;
	struct CarSpawnPos {
		float x, y;
		float yawAng;
	};

	// https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
	// For blue team, flip for orange
	const static CarSpawnPos 
		CAR_SPAWN_LOCATIONS[CAR_SPAWN_LOCATION_AMOUNT] = {
			{ -2048, -2560, M_PI_4 * 1 },
			{  2048, -2560, M_PI_4 * 3 },
			{  -256, -3840, M_PI_4 * 2 },
			{   256, -3840, M_PI_4 * 2 },
			{     0, -4608, M_PI_4 * 2 },
	};

	// https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
	// For blue team, flip for orange
	const static CarSpawnPos // For blue team, flip for orange
		CAR_RESPAWN_LOCATIONS[CAR_RESPAWN_LOCATION_AMOUNT] = {
		{ -2304, -4608, M_PI / 2 },
		{ -2688, -4608, M_PI / 2 },
		{  2304, -4608, M_PI / 2 },
		{  2688, -4608, M_PI / 2 },
	};

	// Input: Forward car speed
	// Output: Max steering angle (radians)
	const static LinearPieceCurve STEER_ANGLE_FROM_SPEED_CURVE = {
		{
			{0,		0.53355587898f},
			{500,	0.31929576934f},
			{1000,	0.182030859666},
			{1500,	0.105695219639},
			{1750,	0.0850682204068},
			{3000,	0.0345432074896}
		}
	};

	// Input: Forward car speed 
	// Output: Extended steering angle (radians)
	const static LinearPieceCurve POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE = {
		{
			{0,		0.39235002f},
			{2500,	0.12610004f},
		}
	};

	// Input: Forward car speed 
	// Output: Torque factor
	const static LinearPieceCurve DRIVE_SPEED_TORQUE_FACTOR_CURVE = {
		{
			{0,		1.0f},
			{1400,	0.1f},
			{1410,	0.0f}
		}
	};

	const static LinearPieceCurve NON_STICKY_FRICTION_FACTOR_CURVE = {
		{
			{0,			0.1f},
			{0.7075f,	0.5f},
			{1,			1.0f}
		}
	};

	const static LinearPieceCurve LAT_FRICTION_CURVE = {
		{
			{0,	1.0f},
			{1,	0.2f},
		}
	};

	const static LinearPieceCurve LONG_FRICTION_CURVE = {
		{
			// Empty curve
		}
	};

	const static LinearPieceCurve HANDBRAKE_LAT_FRICTION_FACTOR_CURVE = {
		{
			{0,	0.1f},
		}
	};

	const static LinearPieceCurve HANDBRAKE_LONG_FRICTION_FACTOR_CURVE = {
		{
			{0,	0.5f},
			{1,	0.9f}
		}
	};

	const static LinearPieceCurve BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE = {
		{
			{     0, 0.65f},
			{ 500.f, 0.65f},
			{2300.f, 0.55f},
			{4600.f, 0.30f}
		}
	};

	constexpr float BUMP_COOLDOWN_TIME = 0.25f;
	constexpr float BUMP_MIN_FORWARD_DIST = 64.5f;

	constexpr float DEMO_RESPAWN_TIME = 3.f;

	const static LinearPieceCurve BUMP_VEL_AMOUNT_GROUND_CURVE = {
		{
			{0.f, (5.f / 6.f)},
			{1400.f, 1100.f},
			{2200.f, 1530.f},
		}
	};

	const static LinearPieceCurve BUMP_VEL_AMOUNT_AIR_CURVE = {
		{
			{0.f, (5.f / 6.f)},
			{1400.f, 1390.f},
			{2200.f, 1945.f},
		}
	};

	const static LinearPieceCurve BUMP_UPWARD_VEL_AMOUNT_CURVE = {
		{
			{0.f, (2.f / 6.f)},
			{1400.f, 278.f},
			{2200.f, 417.f},
		}
	};
}