# Webots controller mrs_uav (spec 09). The binary goes into this folder, as Webots expects.
add_executable(mrs_uav mrs_uav.cpp)
target_link_libraries(mrs_uav PRIVATE mrs_webots mrs_algorithms mrs_warnings)
set_target_properties(mrs_uav PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}" RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}")
# The robot definition, port map and behaviour library from the spec examples.
foreach(f mavic_webots.mrsd mavic_webots.mrsp uav_behaviours.mrsb)
	configure_file("${MRS_SPEC_EXAMPLES}/${f}" "${CMAKE_CURRENT_SOURCE_DIR}/${f}" COPYONLY)
endforeach()
