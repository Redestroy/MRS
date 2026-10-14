# The C API (spec 16 §9): the bench slice. Static by default; MRS_CAPI_SHARED builds a shared library
# for language bindings such as Python (ctypes).
option(MRS_CAPI_SHARED "Build the C API as a shared library" OFF)

if(MRS_CAPI_SHARED)
	set_target_properties(mrs_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
	add_library(mrs_c SHARED src/mrs_c.cpp)
	target_compile_definitions(mrs_c PUBLIC MRS_CAPI_SHARED PRIVATE MRS_CAPI_BUILDING)
	set_target_properties(mrs_c PROPERTIES CXX_VISIBILITY_PRESET hidden)
else()
	add_library(mrs_c STATIC src/mrs_c.cpp)
endif()
target_include_directories(mrs_c PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mrs_c PRIVATE mrs_core mrs_warnings)
