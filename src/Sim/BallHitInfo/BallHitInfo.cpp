#include "BallHitInfo.h"

RS_NS_START

void BallHitInfo::Serialize(DataStreamOut& out) const {
	out.Write<bool>(isValid);

	if (isValid)
		out.WriteMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}

void BallHitInfo::Deserialize(DataStreamIn& in) {
	in.Read<bool>(isValid);

	if (isValid)
		in.ReadMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}

RS_NS_END