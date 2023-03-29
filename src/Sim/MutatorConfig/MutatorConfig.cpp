#include "MutatorConfig.h"

void MutatorConfig::Serialize(DataStreamOut& out) const {
	out.Write<uint16_t>(RS_GET_ARGUMENT_COUNT(MUTATOR_CONFIG_SERIALIZATION_FIELDS));
	out.WriteMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}

void MutatorConfig::Deserialize(DataStreamIn& in) {
	uint16_t argCount = in.Read<uint16_t>();

	if (argCount != RS_GET_ARGUMENT_COUNT(MUTATOR_CONFIG_SERIALIZATION_FIELDS)) {
		RS_ERR_CLOSE(" MutatorConfig::Deserialize(): Mutator config is from a different version of RocketSim, fields don't match!");
	}

	in.ReadMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}