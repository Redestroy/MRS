add_library(mrs_core STATIC
	src/protocol/Record.cpp
	src/protocol/Parser.cpp
	src/protocol/Writer.cpp
	src/device/Action.cpp
	src/device/Params.cpp
	src/device/Capability.cpp
	src/device/DeviceNode.cpp
	src/device/DeviceRegistry.cpp
	src/device/DeviceTree.cpp
	src/device/BlockBuilder.cpp
	src/device/SelfModel.cpp
	src/device/Robot.cpp
	src/device/uav/UavDevices.cpp
	src/device/uav/FlightControlUnit.cpp
	src/port/Port.cpp
	src/port/PortManager.cpp
	src/world/Worldview.cpp
	src/world/GeoReference.cpp
	src/world/ProcessorCatalog.cpp
	src/world/View.cpp
	src/world/Processor.cpp
	src/world/UavProcessors.cpp
	src/world/WorldviewRequirements.cpp
	src/task/Condition.cpp
	src/task/Function.cpp
	src/task/TaskAction.cpp
	src/task/Task.cpp
	src/task/TaskFactory.cpp
	src/task/BehaviourLibrary.cpp
	src/task/TaskExecutor.cpp
	src/robot/Resources.cpp
	src/robot/SafetySupervisor.cpp
	src/robot/TaskJournal.cpp
	src/robot/RobotController.cpp
)
target_include_directories(mrs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_core PRIVATE mrs_warnings)
