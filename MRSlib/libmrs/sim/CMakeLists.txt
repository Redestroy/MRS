# The test simulator and the team harness (spec 10 §4): used by the tests and the batch runner.
add_library(mrs_sim
	src/Team.cpp
	src/TaskSets.cpp
	src/Experiment.cpp
	src/TreeSets.cpp
)
target_include_directories(mrs_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_sim PUBLIC mrs_algorithms mrs_core PRIVATE mrs_generators mrs_warnings)
