cmake_minimum_required(VERSION 3.14)
project(pyquik LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# QuIK headers are header-only except geometry.cpp implementation
set(QUIK_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../quik")

add_library(quik_geometry OBJECT
    ${QUIK_SRC_DIR}/src/geometry.cpp)

target_include_directories(quik_geometry PUBLIC ${QUIK_SRC_DIR}/include)

target_link_libraries(quik_geometry PUBLIC Eigen3::Eigen)

pybind11_add_module(pyquik MODULE bindings.cpp $<TARGET_OBJECTS:quik_geometry>)

target_include_directories(pyquik PRIVATE ${QUIK_SRC_DIR}/include)

target_link_libraries(pyquik PRIVATE Eigen3::Eigen) 

# ------------------------------------------------------------
# Make sure the built shared library ends up next to this CMakeLists
# (i.e. inside numeric_solver/pyquik/) so `import pyquik` works without
# fiddling with PYTHONPATH.

set_target_properties(pyquik PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Also copy on POST_BUILD for multi-config generators to cover all cases
add_custom_command(TARGET pyquik POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pyquik> ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Copying built module to source dir for easy import"
) 