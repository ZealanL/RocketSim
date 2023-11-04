cmake_minimum_required (VERSION 3.8)

project("RocketSim")

# Add all headers and code files
file(GLOB_RECURSE FILES_ROCKETSIM "${PROJECT_SOURCE_DIR}/src/*.cpp" "${PROJECT_SOURCE_DIR}/src/*.h")

# Only include bullet headers when using MSVC, otherwise just code files
file(GLOB_RECURSE FILES_BULLET "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.cpp" "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.h")

add_library(RocketSim ${FILES_ROCKETSIM} ${FILES_BULLET})

set_target_properties(RocketSim PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RocketSim PROPERTIES CXX_STANDARD 20)