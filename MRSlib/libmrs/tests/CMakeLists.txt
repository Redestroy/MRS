add_executable(mrs_tests
	main.cpp
	protocol/test_roundtrip.cpp
	protocol/test_errors.cpp
	protocol/test_parser.cpp
	protocol/test_packing.cpp
	task/test_conditions.cpp
	task/test_executor.cpp
	device/test_devices.cpp
	device/test_identity.cpp
	device/test_parts.cpp
	device/test_writer.cpp
	device/test_providers.cpp
	device/test_check.cpp
	device/test_discovery.cpp
	device/test_bench.cpp
	world/test_worldview.cpp
	flight/test_flight.cpp
	team/test_team.cpp
	sim/test_planner.cpp
	tree/test_tree.cpp
	sta/test_sta.cpp
	cbba/test_cbba.cpp
	sync/test_sync.cpp
	replay/test_replay.cpp
)
target_include_directories(mrs_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
# Include the std headers instead of doctest forward-declaring std::tuple (MSVC warning C5285).
target_compile_definitions(mrs_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
target_link_libraries(mrs_tests PRIVATE mrs_core mrs_algorithms mrs_generators mrs_sim mrs_replay mrs_warnings)
# Test inputs are the spec examples, so the spec and the tests cannot drift apart (spec 07 §4).
target_compile_definitions(mrs_tests PRIVATE MRS_SPEC_EXAMPLES="${MRS_SPEC_EXAMPLES}")
if(TARGET mrs_ardupilot)
	target_sources(mrs_tests PRIVATE ardupilot/test_ardupilot.cpp)
	target_link_libraries(mrs_tests PRIVATE mrs_ardupilot)
endif()
add_test(NAME mrs_tests COMMAND mrs_tests)

# The C API from plain C (spec 16 §9).
if(TARGET mrs_c)
	enable_language(C)
	add_executable(mrs_capi_test capi/test_capi.c)
	target_link_libraries(mrs_capi_test PRIVATE mrs_c)
	if(NOT MRS_CAPI_SHARED)
		# A static C API needs the C++ runtime at link time.
		set_target_properties(mrs_capi_test PROPERTIES LINKER_LANGUAGE CXX)
	endif()
	add_test(NAME mrs_capi_test COMMAND mrs_capi_test)
endif()
