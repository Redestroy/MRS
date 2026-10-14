# MRS::Replay (spec 17): track files and task markers for the MRS dot viewer. No SFML here, so the
# formats build and are tested everywhere; the viewer itself is the optional viewer/ target.
add_library(mrs_replay
	src/Track.cpp
	src/TaskMarkers.cpp
)
target_include_directories(mrs_replay PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_replay PUBLIC mrs_core PRIVATE mrs_algorithms mrs_warnings)
