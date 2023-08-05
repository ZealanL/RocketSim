cmake_minimum_required (VERSION 3.8)

project("RocketSim")

# Add all headers and code files
file(GLOB_RECURSE FILES_ROCKETSIM "${PROJECT_SOURCE_DIR}/src/*.cpp" "${PROJECT_SOURCE_DIR}/src/*.h")

# Only include bullet headers when using MSVC, otherwise just code files
file(GLOB_RECURSE FILES_BULLET "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.cpp" "${PROJECT_SOURCE_DIR}/libsrc/bullet3-3.24/*.h")

add_library(RocketSim ${FILES_ROCKETSIM} ${FILES_BULLET})

set_target_properties(RocketSim PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(RocketSim PROPERTIES CXX_STANDARD 20)

if (TARGET_LANGUAGE STREQUAL "python")
	message(STATUS "Building RocketSim Python bindings using pybind11...")

	# Add pybind11 CMake project
	add_subdirectory("python/pybind11")

	# Get all binding headers
	file(GLOB_RECURSE FILES_ROCKETSIM_PYB "${PROJECT_SOURCE_DIR}/python/src/*.cpp" "${PROJECT_SOURCE_DIR}/python/src/*.h")

	# Create pybind11 module
	pybind11_add_module(rocketsim ${FILES_ROCKETSIM} ${FILES_BULLET} ${FILES_ROCKETSIM_PYB})

	set_target_properties(rocketsim PROPERTIES LINKER_LANGUAGE CXX)
	set_target_properties(rocketsim PROPERTIES CXX_STANDARD 20)
	
	# Set output dir
	set_target_properties(rocketsim PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/python")
	set_target_properties(rocketsim PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASES "${PROJECT_SOURCE_DIR}/python")

	# Enable pybind code with define RS_PYBIND
	target_compile_definitions(rocketsim PUBLIC -DRS_PYBIND)
endif()