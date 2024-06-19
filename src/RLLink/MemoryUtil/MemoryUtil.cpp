#include "MemoryUtil.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#else
#include <sys/mman.h>
#include <cerrno>
#endif

RS_NS_START

// Most of the code in here is modified from https://github.com/ZealanL/RLArenaCollisionDumper/blob/main/src/Memory/Memory.cpp

#define WILDCARD -1

std::vector<int> ParseStringBytePattern(const char* pattern) {

	std::vector<int> result;

	for (const char* s = pattern; *s != NULL; s++) {
		char c = *s;
		if (isblank(c))
			continue;

		if (c == '?') {
			result.push_back(WILDCARD);

			if (*(s + 1) == '?') {
				// Two question marks in a row is still just one wildcard byte
				// Skip this next character so we don't add two wildcards
				s++;
			}
		} else if (isalnum(c)) {
			// Read hex bytes
			result.push_back(strtoul(s, NULL, 16));
			s++; // Go past first character, second will be skipped on next loop
		}
	}

	return result;
}

const byte* MemoryUtil::PatternScan(const byte* moduleMemory, size_t moduleSize, const char* patternStr) {
	std::vector<int> pattern = ParseStringBytePattern(patternStr);
	if (pattern.empty() || pattern[0] == WILDCARD) {
		RS_ERR_CLOSE("MemoryUtil::PatternScan(): Invalid pattern: \"" << patternStr << "\"")
		return NULL; // Invalid pattern
	}

	const byte* binaryStart = moduleMemory;
	const byte* binaryEnd = binaryStart + moduleSize - (pattern.size() - 1);

	int* patternData = pattern.data();
	int patternSize = pattern.size();
	for (const byte* i = binaryStart; i < binaryEnd; i++) {
		bool found = true;
		for (int j = 0; j < patternSize; j++) {
			if (pattern[j] == WILDCARD)
				continue;

			if (pattern[j] != i[j]) {
				found = false;
				break;
			}
		}

		if (found) {
			return i;
		}
	}

	return NULL;
}

void MemoryUtil::SetMemoryProtection(byte* memory, size_t size, bool isCode) {
	constexpr const char* ERR_PREFIX = "MemoryUtil::SetMemoryProtection(): ";

#ifdef _WIN32
	uint32_t _oldProtect;
	if (!VirtualProtect(memory, size, isCode ? PAGE_EXECUTE_READ : PAGE_READWRITE, (PDWORD)&_oldProtect))
		RS_ERR_CLOSE(ERR_PREFIX << "Failed to change memory protection of loaded RocketLeague.exe, error code: " << GetLastError());
#else
	if (mprotect(memory, size, isCode ? (PROT_EXEC | PROT_READ) : (PROT_READ | PROT_WRITE))) {
		RS_ERR_CLOSE(
			ERR_PREFIX << "Failed to change memory protection of loaded RocketLeague.exe!"
			<< "\nError: " << std::strerror(errno)
		);
	}
#endif
}

RS_NS_END