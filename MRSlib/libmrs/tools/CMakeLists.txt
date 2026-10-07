add_executable(mrs_fmt mrs_fmt.cpp)
target_link_libraries(mrs_fmt PRIVATE mrs_core mrs_warnings)

add_executable(mrs_port2021 mrs_port2021.cpp)
target_link_libraries(mrs_port2021 PRIVATE mrs_algorithms mrs_warnings)

# The WP6 batch runner (spec 10 §6).
find_package(Threads REQUIRED)
add_executable(mrs_experiment mrs_experiment.cpp)
target_link_libraries(mrs_experiment PRIVATE mrs_sim Threads::Threads mrs_warnings)
target_compile_definitions(mrs_experiment PRIVATE MRS_SPEC_EXAMPLES="${MRS_SPEC_EXAMPLES}")
