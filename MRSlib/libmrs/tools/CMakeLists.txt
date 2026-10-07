add_executable(mrs_fmt mrs_fmt.cpp)
target_link_libraries(mrs_fmt PRIVATE mrs_core mrs_warnings)

add_executable(mrs_port2021 mrs_port2021.cpp)
target_link_libraries(mrs_port2021 PRIVATE mrs_algorithms mrs_warnings)
