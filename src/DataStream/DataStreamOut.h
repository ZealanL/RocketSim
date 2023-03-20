#pragma once
#include "../BaseInc.h"

#include "SerializeObject.h"

// Basic struct for writing raw data to a file
struct DataStreamOut {
	std::filesystem::path filePath;
	std::ofstream fileStream;
	size_t pos = 0;

	DataStreamOut(std::filesystem::path filePath, bool writeVersionCheck = true) : filePath(filePath) {
		this->fileStream = std::ofstream(filePath, std::ios::binary);
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to write to file " << filePath << ", cannot open file.");

		if (writeVersionCheck)
			Write<uint32_t>(RS_VERSION_ID);
	}

	void WriteBytes(const void* ptr, size_t amount) {
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to write to file " << this->filePath << " during write operation, cannot open file.");

		if (RS_IS_BIG_ENDIAN) {
			byte* reversed = (byte*)malloc(amount);
			memcpy(reversed, ptr, amount);
			std::reverse(reversed, reversed + amount);
			fileStream.write((const char*)reversed, amount);
			free(reversed);
		} else {
			fileStream.write((const char*)ptr, amount);
		}

		pos += amount;
	}

	template <typename T>
	void Write(const T& val) {
		WriteBytes(&val, sizeof(T));
	}

	void WriteMultipleFromList(std::vector<SerializeObject> objs) {
		for (const SerializeObject& obj : objs)
			WriteBytes(obj.ptr, obj.size);
	}

	template<typename... Args>
	void WriteMultiple(Args... args) {
		WriteMultipleFromList({ args... });
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