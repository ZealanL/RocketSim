#pragma once

// AVAILABLE DEFS FOR ROCKETSIM:
//	RS_MAX_SPEED: Define this to remove certain sanity checks for faster speed
//	RS_DONT_LOG: Define this to disable all logging output

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

#define _USE_MATH_DEFINES // for M_PI and similar
#include <cmath>

// Remove need for std namespace scope for very common datatypes
using std::vector;
using std::map;
using std::unordered_map;
using std::set;
using std::multiset;
using std::unordered_set;
using std::list;
using std::stack;
using std::deque;
using std::string;
using std::wstring;
using std::pair;

// Integer typedefs
typedef int8_t	int8;	typedef uint8_t	 uint8;
typedef int16_t int16;	typedef uint16_t uint16;
typedef int32_t int32;	typedef uint32_t uint32;
typedef int64_t int64;	typedef uint64_t uint64;
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

#define RS_STR(s) ([&]{ std::stringstream __macroStream; __macroStream << s; return __macroStream.str(); }())

// Returns sign of number (1 if positive, -1 if negative, and 0 if 0)
#define RS_SGN(val) ((val > 0) - (val < 0))

#define RS_ERR_CLOSE(s) { RS_LOG("FATAL ERROR: " << s); exit(EXIT_FAILURE); }

#if 0 // FOR FUTURE USE: Exports/imports setup
#ifdef ROCKETSIM_EXPORTS
#define RSAPI __declspec(dllexport)
#else
#define RSAPI __declspec(dllimport)
#endif
#else
#define RSAPI
#endif

#define RS_RAND(min, max) (min + (rand() % (max - min)))