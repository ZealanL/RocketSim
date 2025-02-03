#include "CarConfig.h"

RS_NS_START

// Default car-type config definitions

// For those confused about the hitbox numbers,
// as any Google search for car hitbox sizes will conflict with this:
// - They are all wrong. The values that Rocket League gives you
//   when you call the function GetLocalCollisionExtent() are
//   slightly larger than the actual values used in simulation.
//
// Why?
// - Idk lol.
//
// How do I know I'm right?
// -  The hitbox values I have produce a matching inertia matrix with real RL,
//    the ones everyone else uses/shares do not.

const static Vec HITBOX_SIZES[7] = { 
	{ 120.507f,	86.6994f,	38.6591f	}, // OCTANE
	{ 130.427f,	85.7799f,	33.8f		}, // DOMINUS
	{ 131.32f,	87.1704f,	31.8944f	}, // PLANK
	{ 133.992f,	83.021f,	32.8f		}, // BREAKOUT
	{ 129.519f,	84.6879f,	36.6591f	}, // HYBRID
	{ 123.22f,	79.2103f,	44.1591f	}, // MERC

	{ 120.507f + 0.134f, 86.6994f + 0.134f,	38.6591f + 0.134f}   // PSYCLOPS
};

const static Vec HITBOX_OFFSETS[7] = { 
	{ 13.8757f,	0, 20.755f	}, 
	{ 9.f,		0, 15.75f	},	
	{ 9.00857f, 0, 12.0942f	},
	{ 12.5f,	0, 11.75f	},
	{ 13.8757f,	0, 20.755f	},
	{ 11.3757f,	0, 21.505f	},

	{ 13.8757f,	0, 15.0f	}
};

constexpr float FRONT_WHEEL_RADS[7] = {
	12.50f, 12.00f, 12.50f, 13.50f, 12.50f, 15.00f, 
	12.50f
};

constexpr float BACK_WHEEL_RADS[7] = {
	15.00f, 13.50f, 17.00f, 15.00f, 15.00f, 15.00f, 
	15.00f
};

constexpr float FRONT_WHEEL_SUS_REST[7] = {
	38.755f, 33.95f, 31.9242f, 29.7f, 38.755f, 39.505f, 
	33.0f
};

constexpr float BACK_WHEEL_SUS_REST[7] = {
	37.055f, 33.85f, 27.9242f, 29.666f, 37.055f, 39.105f, 
	31.3f
};

const static Vec FRONT_WHEELS_OFFSET[7] = { 
	{ 51.25f, 25.90f, 20.755f	}, 
	{ 50.30f, 31.10f, 15.75f	},
	{ 49.97f, 27.80f, 12.0942f	},
	{ 51.50f, 26.67f, 11.75f	},
	{ 51.25f, 25.90f, 20.755f	},
	{ 51.25f, 25.90f, 21.505f	},

	{ 51.25f, 5.000f, 15.000f	},
};

const static Vec BACK_WHEELS_OFFSET[7] = { 
	{ -33.75f, 29.50f, 20.755f	}, 
	{ -34.75f, 33.00f, 15.75f	},
	{ -35.43f, 20.28f, 12.0942f	},
	{ -35.75f, 35.00f, 11.75f	},
	{ -34.00f, 29.50f, 20.755f	},
	{ -33.75f, 29.50f, 21.505f	},

	{ -33.75f, 29.50f, 15.000f	},
};

// Using a macro here for convenience
#define MAKE_CAR_CONFIG(name, index, threeWheels) \
	const CarConfig CAR_CONFIG_##name = { \
		HITBOX_SIZES[index],   \
		HITBOX_OFFSETS[index], \
		{ FRONT_WHEEL_RADS[index], FRONT_WHEEL_SUS_REST[index], FRONT_WHEELS_OFFSET[index] }, \
		{ BACK_WHEEL_RADS[index], BACK_WHEEL_SUS_REST[index], BACK_WHEELS_OFFSET[index] },    \
		threeWheels \
	}

MAKE_CAR_CONFIG(OCTANE, 0, false);
MAKE_CAR_CONFIG(DOMINUS, 1, false);
MAKE_CAR_CONFIG(PLANK, 2, false);
MAKE_CAR_CONFIG(BREAKOUT, 3, false);
MAKE_CAR_CONFIG(HYBRID, 4, false);
MAKE_CAR_CONFIG(MERC, 5, false);
MAKE_CAR_CONFIG(PSYCLOPS, 6, true);

RS_NS_END