#include "MutatorConfig.h"

void MutatorConfig::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}

void MutatorConfig::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}