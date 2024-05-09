#pragma once
#include "../BaseInc.h"

#include "SerializeObject.h"

RS_NS_START

// Basic struct for reading raw data from a file
struct DataStreamIn {
	std::vector<byte> data;
	size_t pos = 0;

	DataStreamIn() = default;

	DataStreamIn(std::filesystem::path filePath, bool versionCheck) {
		std::ifstream fileStream = std::ifstream(filePath, std::ios::binary);
		if (!fileStream.good())
			RS_ERR_CLOSE("Failed to read file " << filePath << ", cannot open file.");
		
		fileStream >> std::noskipws;
		data = std::vector<byte>(std::istreambuf_iterator<char>(fileStream), std::istreambuf_iterator<char>());

		if (versionCheck && !DoVersionCheck()) {
			RS_ERR_CLOSE("Failed to read file " << filePath << ", file is invalid or from a different version of RocketSim.");
		}
	}

	bool DoVersionCheck() {
		uint32_t versionID = Read<uint32_t>();
		return versionID == RS_VERSION_ID;
	}

	bool IsDone() const {
		return pos >= data.size();
	}

	bool IsOverflown() const {
		return pos > data.size();
	}

	size_t GetNumBytesLeft() const {
		if (IsDone()) {
			return 0;
		} else {
			return data.size() - pos;
		}
	}

	void ReadBytes(void* out, size_t amount) {
		if (GetNumBytesLeft() >= amount) {
			byte* asBytes = (byte*)out;
			memcpy(asBytes, data.data() + pos, amount);

			if (RS_IS_BIG_ENDIAN)
				std::reverse(asBytes, asBytes + sizeof(amount));
		}

		pos += amount;
	}

	template <typename T>
	T Read() {
		byte bytes[sizeof(T)];
		ReadBytes(bytes, sizeof(T));
		return *(T*)bytes;
	}

	template <typename T>
	void Read(T& out) {
		out = Read<T>();
	}

	void ReadMultipleFromList(std::vector<SerializeObject> objs) {
		uint32_t amount = Read<uint32_t>();
		if (amount != objs.size())
			RS_ERR_CLOSE("DataStreamIn::ReadMultipleFromList(): Prop count mismatch, expected " << objs.size() << " but have " << amount << ".");
		
		for (const SerializeObject& obj : objs)
			ReadBytes(obj.ptr, obj.size);
	}

	template <typename... Args>
	void ReadMultiple(Args&... args) {
		ReadMultipleFromList({ args... });
	}
};

RS_NS_END