#include "CarConfig.h"

// Default car-type config definitions
// Thanks to https://cars.rocketscience.fyi for this 90% of this data (I am very lazy)
// Only thing I needed to get myself was the suspension rest lengths

const static Vec HITBOX_SIZES[6] = {
	{ 118.01f, 84.20f, 36.16f },
	{ 127.93f, 83.28f, 31.30f },
	{ 128.82f, 84.67f, 29.39f },
	{ 131.49f, 80.52f, 30.30f },
	{ 127.02f, 82.19f, 34.16f },
	{ 120.72f, 76.71f, 41.66f }
};

const static Vec HITBOX_OFFSETS[6] = {
	{ 13.88f, 0, 20.75f },
	{ 9.00f,  0, 15.75f },
	{ 9.01f,  0, 12.09f },
	{ 12.50f, 0, 11.75f },
	{ 13.88f, 0, 20.75f },
	{ 11.38f, 0, 21.50f }
};

constexpr float FRONT_WHEEL_RADS[6] = {
	12.50f,
	12.00f,
	12.50f,
	13.50f,
	12.50f,
	15.00f,
};

constexpr float BACK_WHEEL_RADS[6] = {
	15.00f,
	13.50f,
	17.00f,
	15.00f,
	15.00f,
};

// whhen the impoter is sus
constexpr float FRONT_WHEEL_SUS_REST[6] = {
	38.755f,
	33.95f,
	31.9242f,
	29.7f,
	38.755f,
};

constexpr float BACK_WHEEL_SUS_REST[6] = {
	37.055f,
	33.85f,
	27.9242f,
	29.666f,
	37.055f,
};

const static Vec FRONT_WHEELS_OFFSET[6] = {
	{ 51.25f, 25.90f, -6.00f },
	{ 50.30f, 31.10f, -6.20f },
	{ 49.97f, 27.80f, -7.83f },
	{ 51.50f, 26.67f, -5.95f },
	{ 51.25f, 25.90f, -6.00f },
	{ 51.25f, 25.90f, -6.00f }
};

const static Vec BACK_WHEELS_OFFSET[6] = {
	{ -33.75f, 29.50f, -4.30f  },
	{ -34.75f, 33.00f, -6.10f  },
	{ -35.43f, 20.28f, -3.83f  },
	{ -35.75f, 35.00f, -5.91f  },
	{ -34.00f, 29.50f, -4.30f  },
	{ -33.75f, 29.50f, -5.60f  }
};

// Using a macro here for convenience
#define MAKE_CAR_CONFIG(name, index)			\
const CarConfig CAR_CONFIG_##name = {			\
	HITBOX_SIZES[index], HITBOX_OFFSETS[index], \
	{ FRONT_WHEEL_RADS[index],	FRONT_WHEEL_SUS_REST[index],	FRONT_WHEELS_OFFSET[index]	}, \
	{ BACK_WHEEL_RADS[index],	BACK_WHEEL_SUS_REST[index],		BACK_WHEELS_OFFSET[index]	}, \
}

MAKE_CAR_CONFIG( OCTANE,	0 );
MAKE_CAR_CONFIG( DOMINUS,	1 );
MAKE_CAR_CONFIG( PLANK,		2 );
MAKE_CAR_CONFIG( BREAKOUT,	3 );
MAKE_CAR_CONFIG( HYBRID,	4 );
MAKE_CAR_CONFIG( MERC,		5 );