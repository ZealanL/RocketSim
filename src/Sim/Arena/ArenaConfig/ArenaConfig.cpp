#include "ArenaConfig.h"

RS_NS_START

void ArenaConfig::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(ARENA_CONFIG_SERIALIZATION_FIELDS);

	out.Write(useCustomBoostPads);
	if (useCustomBoostPads) {
		out.Write<uint32_t>(customBoostPads.size());
		for (auto& boostPad : customBoostPads)
			boostPad.Serialize(out);
	}
}

void ArenaConfig::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(ARENA_CONFIG_SERIALIZATION_FIELDS);

	useCustomBoostPads = in.Read<bool>();
	if (useCustomBoostPads) {
		uint32_t numPads = in.Read<uint32_t>();

		// NOTE: Not reserving/resizing customBoostPads from numPads just in case its a horrible corrupted value

		for (uint32_t i = 0; i < numPads; i++) {
			BoostPadConfig config;
			config.Deserialize(in);

			if (in.IsOverflown())
				RS_ERR_CLOSE("Overflow after reading custom boost (" << (i + 1) << " / " << numPads << ")");

			customBoostPads.push_back(config);
		}
	}
}

RS_NS_END