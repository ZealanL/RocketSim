#include "MutatorConfig.h"

RS_NS_START

MutatorConfig::MutatorConfig(GameMode gameMode) {
	using namespace RLConst;

	switch (gameMode) {
	case GameMode::HOOPS:
		ballRadius = BALL_COLLISION_RADIUS_HOOPS;
		break;
	case GameMode::SNOWDAY:
		ballRadius = Snowday::PUCK_RADIUS;
		break;
	default:
		ballRadius = BALL_COLLISION_RADIUS_SOCCAR;
	}

	if (gameMode == GameMode::SNOWDAY) {
		ballWorldFriction = Snowday::PUCK_FRICTION;
		ballWorldRestitution = Snowday::PUCK_RESTITUTION;
		ballMass = Snowday::PUCK_MASS_BT;
	} else {
		ballWorldFriction = BALL_FRICTION;
		ballWorldRestitution = BALL_RESTITUTION;
		ballMass = BALL_MASS_BT;
	}

	if (gameMode == GameMode::HEATSEEKER) {
		// Infinite boost
		carSpawnBoostAmount = 100;
		boostUsedPerSecond = 0;
	}
}

void MutatorConfig::Serialize(DataStreamOut& out) const {
	out.Write<uint16_t>(RS_GET_ARGUMENT_COUNT(MUTATOR_CONFIG_SERIALIZATION_FIELDS));
	out.WriteMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}

void MutatorConfig::Deserialize(DataStreamIn& in) {
	uint16_t argCount = in.Read<uint16_t>();

	if (argCount != RS_GET_ARGUMENT_COUNT(MUTATOR_CONFIG_SERIALIZATION_FIELDS)) {
		RS_ERR_CLOSE(" MutatorConfig::Deserialize(): Mutator config is from a different version of RocketSim, fields don't match");
	}

	in.ReadMultiple(MUTATOR_CONFIG_SERIALIZATION_FIELDS);
}

RS_NS_END