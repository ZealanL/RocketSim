#pragma once
#include "../../Framework.h"

RS_NS_START

namespace MemoryUtil {
	// Finds a pattern of bytes in a block of memory, supports wildcards
	// Pattern string example: "0F 29 74 24 ? 0F 28 F1 FF 90 ? ? ? ? 48"
	// Returns offset from module base
	const byte* PatternScan(const byte* moduleMemory, size_t moduleSize, const char* patternStr);

	void SetMemoryProtection(byte* memory, size_t size, bool isCode);
}

RS_NS_END