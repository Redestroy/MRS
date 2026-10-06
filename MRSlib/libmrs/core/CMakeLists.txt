add_library(mrs_core STATIC
	src/protocol/Record.cpp
	src/protocol/Parser.cpp
	src/protocol/Writer.cpp
	src/device/Action.cpp
	src/world/Worldview.cpp
	src/world/GeoReference.cpp
	src/task/Condition.cpp
	src/task/Function.cpp
	src/task/TaskAction.cpp
	src/task/Task.cpp
	src/task/TaskFactory.cpp
	src/task/BehaviourLibrary.cpp
	src/task/TaskExecutor.cpp
)
target_include_directories(mrs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_core PRIVATE mrs_warnings)
