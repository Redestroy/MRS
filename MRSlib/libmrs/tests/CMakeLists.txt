add_executable(mrs_tests
	main.cpp
	protocol/test_smoke.cpp
)
target_include_directories(mrs_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
target_link_libraries(mrs_tests PRIVATE mrs_core mrs_warnings)
# Test inputs are the spec examples, so the spec and the tests cannot drift apart (spec 07 §4).
get_filename_component(MRS_SPEC_EXAMPLES "${CMAKE_CURRENT_SOURCE_DIR}/../../../MRS_Projects/Spatial3D/spec/examples" ABSOLUTE)
target_compile_definitions(mrs_tests PRIVATE MRS_SPEC_EXAMPLES="${MRS_SPEC_EXAMPLES}")
add_test(NAME mrs_tests COMMAND mrs_tests)
