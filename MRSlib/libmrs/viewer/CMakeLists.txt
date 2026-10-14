# The MRS dot viewer (spec 17 §4): an SFML window that replays a track file with its task files.
# Optional (MRS_BUILD_VIEWER, off by default) because it fetches and builds SFML; CI builds the
# library without it. SFML is linked statically, so the program runs without SFML DLLs.
include(FetchContent)
set(MRS_SFML_VERSION 3.0.2 CACHE STRING "SFML tag the viewer fetches")
set(BUILD_SHARED_LIBS OFF)
set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.25)
	set(MRS_SFML_SYSTEM SYSTEM)  # SFML's headers do not trip our warnings-as-errors
endif()
FetchContent_Declare(SFML
	GIT_REPOSITORY https://github.com/SFML/SFML.git
	GIT_TAG ${MRS_SFML_VERSION}
	GIT_SHALLOW ON
	EXCLUDE_FROM_ALL
	${MRS_SFML_SYSTEM})
FetchContent_MakeAvailable(SFML)

add_executable(mrs_dot_viewer
	src/main.cpp
	src/Canvas.cpp
)
# A console program, so warnings about task files show where it was started.
target_link_libraries(mrs_dot_viewer PRIVATE mrs_replay SFML::Graphics mrs_warnings)
