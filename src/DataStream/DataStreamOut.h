#pragma once
#include "../BaseInc.h"

#include "SerializeObject.h"

RS_NS_START

// Basic struct for writing raw data to a file
struct DataStreamOut {
	std::vector<byte> data;
	size_t pos = 0;

	DataStreamOut() = default;

	void WriteBytes(const void* ptr, size_t amount) {
		data.reserve(amount);
		if (RS_IS_BIG_ENDIAN) {
			byte* reversed = (byte*)malloc(amount);
			memcpy(reversed, ptr, amount);
			std::reverse(reversed, reversed + amount);
			data.insert(data.end(), (byte*)reversed, (byte*)reversed + amount);
			free(reversed);
		} else {
			data.insert(data.end(), (byte*)ptr, (byte*)ptr + amount);
		}

		pos += amount;
	}

	template <typename T>
	void Write(const T& val) {
		WriteBytes(&val, sizeof(T));
	}

	void WriteMultipleFromList(std::vector<SerializeObject> objs) {
		Write<uint32_t>(objs.size());
		for (const SerializeObject& obj : objs)
			WriteBytes(obj.ptr, obj.size);
	}

	template<typename... Args>
	void WriteMultiple(Args... args) {
		WriteMultipleFromList({ args... });
	}

	void WriteToFile(std::filesystem::path filePath, bool writeVersionCheck) {
		std::ofstream fileStream = std::ofstream(filePath, std::ios::binary);
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to write to file " << filePath << ", cannot open file.");

		if (writeVersionCheck) {
			uint32_t version = RS_VERSION_ID;
			byte* versionBytes = (byte*)&version;
			data.insert(data.begin(), versionBytes, versionBytes + sizeof(version));
		}

		if (!data.empty())
			fileStream.write((char*)data.data(), data.size());
	}
};

template <>
inline void DataStreamOut::Write(const Vec& val) {
	WriteMultiple(val.x, val.y, val.z);
}

template <>
inline void DataStreamOut::Write(const RotMat& val) {
	WriteMultiple(val.forward, val.right, val.up);
}

RS_NS_END