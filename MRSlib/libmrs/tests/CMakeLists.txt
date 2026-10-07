add_executable(mrs_tests
	main.cpp
	protocol/test_roundtrip.cpp
	protocol/test_errors.cpp
	protocol/test_parser.cpp
	protocol/test_packing.cpp
	task/test_conditions.cpp
	task/test_executor.cpp
	device/test_devices.cpp
	world/test_worldview.cpp
	flight/test_flight.cpp
	team/test_team.cpp
)
target_include_directories(mrs_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
# Include the std headers instead of doctest forward-declaring std::tuple (MSVC warning C5285).
target_compile_definitions(mrs_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
target_link_libraries(mrs_tests PRIVATE mrs_core mrs_algorithms mrs_warnings)
# Test inputs are the spec examples, so the spec and the tests cannot drift apart (spec 07 §4).
target_compile_definitions(mrs_tests PRIVATE MRS_SPEC_EXAMPLES="${MRS_SPEC_EXAMPLES}")
add_test(NAME mrs_tests COMMAND mrs_tests)
