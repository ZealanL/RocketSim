#pragma once
#include "../BaseInc.h"

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

	template <typename T>
	void Write(const T& val) {
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to write to file " << this->filePath << " during write operation, cannot open file.");
		
		if (RS_IS_BIG_ENDIAN) {
			byte reversed[sizeof(T)];
			memcpy(reversed, &val, sizeof(T));
			std::reverse(reversed, reversed + sizeof(T));
			fileStream.write((const char*)reversed, sizeof(T));
		} else {
			fileStream.write((const char*)&val, sizeof(T));
		}
		
		pos += sizeof(T);
	}

	template<typename... Args>
	void WriteMultiple(Args... args) {
		https://stackoverflow.com/questions/12030538/calling-a-function-for-each-variadic-template-argument-and-an-array
		[](...) {}((Write(std::forward<Args>(args)), 0)...);
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