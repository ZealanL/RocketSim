#pragma once

#define RS_VERSION "2.1.1"

#include <stdint.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <queue>
#include <deque>
#include <stack>
#include <cassert>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <list>
#include <functional>
#include <chrono>
#include <filesystem>
#include <random>
#include <mutex>
#include <bit>
#include <thread>
#include <cstring>
#include <array>

#define _USE_MATH_DEFINES // for M_PI and similar
#include <cmath>
#include <math.h>

#ifdef _MSC_VER
// Disable annoying truncation warnings on MSVC
#pragma warning(disable: 4305 4244 4267)
#endif

typedef uint8_t byte;

// Current millisecond time
#define RS_CUR_MS() (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count())

#define RS_MAX(a, b) ((a > b) ? a : b)
#define RS_MIN(a, b) ((a < b) ? a : b)

#define RS_CLAMP(val, min, max) RS_MIN(RS_MAX(val, min), max)

#ifndef RS_DONT_LOG
#define RS_LOG(s) { std::cout << std::dec << s << std::endl; }
#else
#define RS_LOG(s) {}
#endif

#define RS_LOG_BLANK() RS_LOG("")

#define RS_STR(s) ([&]{ std::stringstream __macroStream; __macroStream << s; return __macroStream.str(); }())

// Returns sign of number (1 if positive, -1 if negative, and 0 if 0)
#define RS_SGN(val) ((val > 0) - (val < 0))

#define RS_WARN(s) RS_LOG("ROCKETSIM WARNING: " << s)

#define RS_ERR_CLOSE(s) { \
	std::string _errorStr = RS_STR("ROCKETSIM FATAL ERROR: " << s); \
	RS_LOG(_errorStr); \
	throw std::runtime_error(_errorStr); \
	exit(EXIT_FAILURE); \
}

#if 0 // FOR FUTURE USE: Exports/imports setup
#ifdef ROCKETSIM_EXPORTS
#define RSAPI __declspec(dllexport)
#else
#define RSAPI __declspec(dllimport)
#endif
#else
#define RSAPI
#endif

#define RS_ALIGN_16 alignas(16)

#ifndef RS_NO_NAMESPACE
#define RS_NS_START namespace RocketSim {
#define RS_NS_END }
#else
#define RS_NS_START
#define RS_NS_END
#endif

template<typename ...Args>
size_t __RS_GET_ARGUMENT_COUNT(Args ...) {
	return sizeof...(Args);
}
#define RS_GET_ARGUMENT_COUNT __RS_GET_ARGUMENT_COUNT

constexpr uint32_t __RS_GET_VERSION_ID() {
	uint32_t result = 0;
	for (int i = 0; i < sizeof(RS_VERSION); i++)
		result = RS_MAX(RS_VERSION[i] - '0' + 1, 0) + (result*10);
	return result;
}
#define RS_VERSION_ID (__RS_GET_VERSION_ID())

#define RS_IS_BIG_ENDIAN (std::endian::native == std::endian::big)

// TODO: Remove more permanently
#define RS_NO_SUSPCOLGRID