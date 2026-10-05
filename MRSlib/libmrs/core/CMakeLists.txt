add_library(mrs_core STATIC
	src/protocol/Record.cpp
	src/protocol/Parser.cpp
	src/protocol/Writer.cpp
	src/device/Action.cpp
)
target_include_directories(mrs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_core PRIVATE mrs_warnings)
