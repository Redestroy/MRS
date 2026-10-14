# Webots controller for the WP3 ground-truth check. The binary goes into this folder, as Webots expects.
add_executable(mrs_worldview_check mrs_worldview_check.cpp)
target_link_libraries(mrs_worldview_check PRIVATE mrs_webots mrs_warnings)
set_target_properties(mrs_worldview_check PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}" RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}")
# The robot definition from the spec examples.
configure_file("${MRS_SPEC_EXAMPLES}/mavic_webots.mrsd"
               "${CMAKE_CURRENT_SOURCE_DIR}/mavic_webots.mrsd" COPYONLY)
