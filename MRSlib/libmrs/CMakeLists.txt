cmake_minimum_required(VERSION 3.20)
project(libmrs VERSION 0.1.0 LANGUAGES CXX)

# New MRS library (spec: MRS_Projects/Spatial3D/spec, especially 07_Build_and_Test.md).
option(MRS_BUILD_TESTS "Build the mrs_tests executable" ON)
option(MRS_BUILD_TOOLS "Build command-line tools such as mrs_fmt" ON)
option(MRS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors (on in CI)" OFF)
if(DEFINED ENV{WEBOTS_HOME})
	set(MRS_WEBOTS_DEFAULT ON)
else()
	set(MRS_WEBOTS_DEFAULT OFF)
endif()
option(MRS_BUILD_WEBOTS "Build the Webots platform (needs WEBOTS_HOME)" ${MRS_WEBOTS_DEFAULT})
option(MRS_BUILD_ARDUPILOT "Build the ArduPilot platform (MAVLink, UDP; spec 14)" ON)

# Spec examples: test data and the robot files the Webots controllers copy (spec 07 §4).
get_filename_component(MRS_SPEC_EXAMPLES "${CMAKE_CURRENT_SOURCE_DIR}/../../MRS_Projects/Spatial3D/spec/examples" ABSOLUTE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(mrs_warnings INTERFACE)
if(MSVC)
	target_compile_options(mrs_warnings INTERFACE /W4 /permissive- /utf-8)
	if(MRS_WARNINGS_AS_ERRORS)
		target_compile_options(mrs_warnings INTERFACE /WX)
	endif()
else()
	target_compile_options(mrs_warnings INTERFACE -Wall -Wextra -Wpedantic)
	if(MRS_WARNINGS_AS_ERRORS)
		target_compile_options(mrs_warnings INTERFACE -Werror)
	endif()
endif()

add_subdirectory(core)
add_subdirectory(algorithms)
add_subdirectory(generators)
add_subdirectory(sim)

if(MRS_BUILD_WEBOTS)
	add_subdirectory(platforms/webots)
endif()

if(MRS_BUILD_ARDUPILOT)
	add_subdirectory(platforms/ardupilot)
endif()

if(MRS_BUILD_TOOLS)
	add_subdirectory(tools)
endif()

if(MRS_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()
