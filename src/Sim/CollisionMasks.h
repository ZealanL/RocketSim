#pragma once
#include "../Framework.h"

// Collision masks for different types of objects
// Used so that the net in hoops doesn't collide with the cars
enum CollisionMasks : uint32_t {
	HOOPS_NET = (1 << 8)
};