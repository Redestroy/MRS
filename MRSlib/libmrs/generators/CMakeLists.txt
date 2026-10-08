# MRS::Generators (spec 11 §3-4): GeoJSON input and the task generators.
add_library(mrs_generators
	src/GeoJson.cpp
	src/Generators.cpp
)
target_include_directories(mrs_generators PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_generators PUBLIC mrs_core PRIVATE mrs_warnings)
