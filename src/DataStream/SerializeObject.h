#pragma once
#include "../Framework.h"

RS_NS_START

struct SerializeObject {
	void* ptr;
	size_t size = -1;

	template<typename T>
	SerializeObject(T& val) {
		ptr = &val;
		size = sizeof(T);
	}

	// TODO: Override for Vec/RotMat so that we don't write/read the always-0 fourth component

	SerializeObject(const SerializeObject& other) {
		ptr = other.ptr;
		size = other.size;
	}
};

RS_NS_END