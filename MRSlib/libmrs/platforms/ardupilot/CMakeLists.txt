# ArduPilot platform (spec 14): MAVLink to a flight controller, UDP between agents.
# MAVLink's generated C headers (c_library_v2, common dialect) are vendored in third_party/mavlink;
# MRS_MAVLINK_DIR may point at another copy (a folder holding common/mavlink.h).
set(MRS_MAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink" CACHE PATH "MAVLink C headers (c_library_v2)")
set(mrs_mavlink_include "${MRS_MAVLINK_DIR}")
if(NOT EXISTS "${mrs_mavlink_include}/common/mavlink.h")
	message(FATAL_ERROR "MAVLink headers not found in ${mrs_mavlink_include} (set MRS_MAVLINK_DIR)")
endif()
add_library(mrs_mavlink INTERFACE)
target_include_directories(mrs_mavlink SYSTEM INTERFACE "${mrs_mavlink_include}")

add_library(mrs_ardupilot
	src/Net.cpp
	src/UdpTransport.cpp
	src/ArduPilotPlatform.cpp
	src/GuidedFcu.cpp
	src/MockAutopilot.cpp
)
target_include_directories(mrs_ardupilot PUBLIC include PRIVATE src)
target_link_libraries(mrs_ardupilot PUBLIC mrs_core PRIVATE mrs_mavlink mrs_warnings)
if(WIN32)
	target_link_libraries(mrs_ardupilot PUBLIC ws2_32)
endif()

if(MRS_BUILD_TOOLS)
	add_executable(mrs_ardupilot_uav tools/mrs_ardupilot_uav.cpp)
	target_link_libraries(mrs_ardupilot_uav PRIVATE mrs_ardupilot mrs_algorithms mrs_warnings)
	add_executable(mrs_operator tools/mrs_operator.cpp)
	target_link_libraries(mrs_operator PRIVATE mrs_ardupilot mrs_algorithms mrs_warnings)
	target_compile_definitions(mrs_ardupilot_uav PRIVATE MRS_SPEC_EXAMPLES="${MRS_SPEC_EXAMPLES}")
endif()
