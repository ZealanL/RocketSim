#include "BallHitInfo.h"

void BallHitInfo::Serialize(DataStreamOut& out) {
	out.Write<bool>(isValid);

	if (isValid)
		out.WriteMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}

void BallHitInfo::Deserialize(DataStreamIn& in) {
	in.Read<bool>(isValid);

	if (isValid)
		in.ReadMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}