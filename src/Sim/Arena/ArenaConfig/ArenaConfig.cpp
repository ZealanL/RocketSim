#include "ArenaConfig.h"

RS_NS_START

void ArenaConfig::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(ARENA_CONFIG_SERIALIZATION_FIELDS);
}

void ArenaConfig::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(ARENA_CONFIG_SERIALIZATION_FIELDS);
}

RS_NS_END