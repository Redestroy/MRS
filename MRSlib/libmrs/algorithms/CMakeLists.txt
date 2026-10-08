add_library(mrs_algorithms
	src/TaskPool.cpp
	src/Allocator.cpp
	src/MrsLayer.cpp
	src/TaskIssuer.cpp
	src/Planner.cpp
	src/TaskTree.cpp
	src/Cbba.cpp
	src/Ldta2.cpp
)
target_include_directories(mrs_algorithms PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_algorithms PUBLIC mrs_core PRIVATE mrs_warnings)
