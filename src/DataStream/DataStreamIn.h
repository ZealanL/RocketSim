#pragma once
#include "../BaseInc.h"

// Basic struct for reading raw data from a file
struct DataStreamIn {
	vector<char> data;
	size_t pos = 0;

	DataStreamIn(std::filesystem::path filePath, bool versionCheck = true) {
		std::ifstream fileStream = std::ifstream(filePath, std::ios::binary);
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to read file " << filePath << ", cannot open file.");

		data = vector<char>(std::istreambuf_iterator<char>(fileStream), std::istreambuf_iterator<char>());

		if (versionCheck) {
			uint32_t versionID = Read<uint32_t>();
			if (versionID != RS_VERSION_ID)
				RS_ERR_CLOSE("Failed to read file " << filePath << ", file is invalid or from a different version of RocketSim.");
		}
	}

	bool IsDone() {
		return pos >= data.size();
	}

	bool IsOverflown() {
		return pos > data.size();
	}

	size_t GetNumBytesLeft() {
		if (IsDone()) {
			return 0;
		} else {
			return data.size() - pos;
		}
	}

	template <typename T>
	T Read() {
		if (GetNumBytesLeft() >= sizeof(T)) {
			byte bytes[sizeof(T)];
			memcpy(bytes, data.data() + pos, sizeof(T));
			
			if (RS_IS_BIG_ENDIAN)
				std::reverse(bytes, bytes + sizeof(T));

			pos += sizeof(T);
			return *(T*)bytes;
		} else {
			pos += sizeof(T);
			return T{};
		}
	}

	template <typename T>
	void Read(T& out) {
		out = Read<T>();
	}

	template<typename... Args>
	void ReadMultiple(Args&... args) {
		https://stackoverflow.com/questions/12030538/calling-a-function-for-each-variadic-template-argument-and-an-array
		[](...) {}((Read(std::forward<Args&>(args)), 0)...);
	}

};

template <>
inline Vec DataStreamIn::Read() {
	Vec result;
	ReadMultiple(result.x, result.y, result.z);
	return result;
}

template <>
inline RotMat DataStreamIn::Read() {
	RotMat result;
	ReadMultiple(result.forward, result.right, result.up);
	return result;
}