#pragma once
#include "../../BaseInc.h"

#include "../Car/Car.h"

#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

RS_NS_START

struct BoostPadConfig {
	Vec pos;
	bool isBig = false;

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};
#define BOOSTPADCONFIG_SERIALIZATION_FIELDS \
pos, isBig

struct BoostPadState {
	bool isActive = true;
	float cooldown = 0;

	Car* curLockedCar = NULL;
	uint32_t prevLockedCarID = NULL;

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};
#define BOOSTPAD_SERIALIZATION_FIELDS \
isActive, cooldown, prevLockedCarID

class BoostPad {
public:
	BoostPadConfig config;

	Vec _posBT;
	Vec _boxMinBT, _boxMaxBT;

	BoostPadState _internalState;

	RSAPI BoostPadState GetState() const { return _internalState; }
	RSAPI void SetState(const BoostPadState& state) { _internalState = state; }

	// For construction by Arena
	static BoostPad* _AllocBoostPad();
	void _Setup(const BoostPadConfig& config);

	void _CheckCollide(Car* car);

	void _PreTickUpdate(float tickTime);
	void _PostTickUpdate(float tickTime, const MutatorConfig& mutatorConfig);
private:
	BoostPad() {}
};

RS_NS_END