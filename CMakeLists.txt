cmake_minimum_required (VERSION 3.8)

project("RocketSim")

# Add all headers and code files
file(GLOB_RECURSE FILES_CPP "src/*.cpp")
file(GLOB_RECURSE FILES_H "src/*.h")

# Only include bullet headers when using MSVC, otherwise just code files
if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  file(GLOB FILES_BULLET "libsrc/bullet3-3.24/*.cpp" "libsrc/bullet3-3.24/*.h")
else()
  file(GLOB FILES_BULLET "libsrc/bullet3-3.24/*.cpp")
endif()

add_library(RocketSim ${FILES_CPP} ${FILES_H} ${FILES_BULLET})

set_target_properties(RocketSim PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RocketSim PROPERTIES CXX_STANDARD 17)