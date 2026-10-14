# Webots controller mrs_issuer (spec 09). The binary goes into this folder, as Webots expects.
add_executable(mrs_issuer mrs_issuer.cpp)
target_link_libraries(mrs_issuer PRIVATE mrs_webots mrs_algorithms mrs_warnings)
set_target_properties(mrs_issuer PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}" RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}")
