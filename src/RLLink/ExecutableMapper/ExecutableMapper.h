#pragma once
#include "../../Framework.h"

RS_NS_START

// Window executable mapper for RocketSim that doesn't require windows
namespace ExecutableMapper {
	void Map(byte* exeFileMem, size_t exeFileSize, byte*& memoryOut, size_t& sizeOut, byte*& codeBaseOut, size_t& codeSizeOut);
}

RS_NS_END