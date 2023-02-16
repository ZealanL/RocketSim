cmake_minimum_required (VERSION 3.8)

project("RocketSim")

# Add all headers and code files
file(GLOB_RECURSE SOURCEFILES "src/*.cpp" "libsrc/*.cpp" "src/*.cc" "libsrc/*.cc")
file(GLOB_RECURSE HEADERFILES "src/*.h" "libsrc/*.h")

add_library(RocketSim ${SOURCEFILES} ${HEADERFILES})

set_target_properties(RocketSim PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RocketSim PROPERTIES CXX_STANDARD 17)