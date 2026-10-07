# WebotsPlatform: built only when WEBOTS_HOME points at a Webots installation (spec 07).
# Forward slashes: a Windows path such as C:\Program Files\Webots breaks CMake string escapes.
file(TO_CMAKE_PATH "$ENV{WEBOTS_HOME}" WEBOTS_HOME)
if(APPLE)
	set(MRS_WEBOTS_LIB "${WEBOTS_HOME}/Contents/lib/controller")
	set(MRS_WEBOTS_INCLUDE "${WEBOTS_HOME}/Contents/include/controller")
else()
	set(MRS_WEBOTS_LIB "${WEBOTS_HOME}/lib/controller")
	set(MRS_WEBOTS_INCLUDE "${WEBOTS_HOME}/include/controller")
endif()

find_library(MRS_WEBOTS_C_CONTROLLER NAMES Controller PATHS "${MRS_WEBOTS_LIB}" NO_DEFAULT_PATH REQUIRED)
if(MSVC)
	# Webots ships its C++ API for Windows only as a MinGW build, which MSVC and clang-cl cannot link.
	# Compile the C++ wrapper sources instead, as the Webots guide says for Visual Studio.
	file(GLOB MRS_WEBOTS_CPP_SOURCES "${WEBOTS_HOME}/src/controller/cpp/*.cpp")
	if(NOT MRS_WEBOTS_CPP_SOURCES)
		message(FATAL_ERROR "No Webots C++ API sources in ${WEBOTS_HOME}/src/controller/cpp")
	endif()
	add_library(mrs_webots_cpp STATIC ${MRS_WEBOTS_CPP_SOURCES})
	target_include_directories(mrs_webots_cpp SYSTEM PUBLIC "${MRS_WEBOTS_INCLUDE}/c" "${MRS_WEBOTS_INCLUDE}/cpp")
	target_link_libraries(mrs_webots_cpp PUBLIC ${MRS_WEBOTS_C_CONTROLLER})
	set(MRS_WEBOTS_CPP_CONTROLLER mrs_webots_cpp)
else()
	find_library(MRS_WEBOTS_CPP_CONTROLLER NAMES CppController PATHS "${MRS_WEBOTS_LIB}" NO_DEFAULT_PATH REQUIRED)
endif()

add_library(mrs_webots STATIC src/WebotsPlatform.cpp)
target_include_directories(mrs_webots PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(mrs_webots SYSTEM PUBLIC "${MRS_WEBOTS_INCLUDE}/c" "${MRS_WEBOTS_INCLUDE}/cpp")
target_link_libraries(mrs_webots PUBLIC mrs_core ${MRS_WEBOTS_CPP_CONTROLLER} ${MRS_WEBOTS_C_CONTROLLER} PRIVATE mrs_warnings)

option(MRS_BUILD_WEBOTS_CONTROLLERS "Build the Webots check controllers" ON)
if(MRS_BUILD_WEBOTS_CONTROLLERS)
	add_subdirectory(controllers/mrs_worldview_check)
	add_subdirectory(controllers/mrs_uav)
	add_subdirectory(controllers/mrs_issuer)
endif()
