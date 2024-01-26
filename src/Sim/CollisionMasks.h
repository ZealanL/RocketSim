#pragma once
#include "../Framework.h"

RS_NS_START

// Collision masks for different types of objects
// Used so that the net in hoops doesn't collide with the cars
enum CollisionMasks : uint32_t {
	HOOPS_NET = (1 << 8)
};

RS_NS_END