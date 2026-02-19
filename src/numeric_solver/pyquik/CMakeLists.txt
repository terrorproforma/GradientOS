cmake_minimum_required(VERSION 3.14)
project(pyquik LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(FetchContent)

find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
  message(STATUS "pybind11 not found via find_package; fetching source.")
  FetchContent_Declare(
    pybind11
    URL https://github.com/pybind/pybind11/archive/refs/tags/v2.13.6.tar.gz
  )
  FetchContent_MakeAvailable(pybind11)
endif()

set(EIGEN3_INCLUDE_DIR "")
set(HAVE_EIGEN3_PACKAGE OFF)
find_package(Eigen3 CONFIG QUIET)
if(Eigen3_FOUND)
  set(HAVE_EIGEN3_PACKAGE ON)
  message(STATUS "Found Eigen3 via find_package.")
else()
  message(STATUS "Eigen3 not found via find_package; fetching source.")
  FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  )
  FetchContent_MakeAvailable(eigen)
  set(EIGEN3_INCLUDE_DIR "${eigen_SOURCE_DIR}")
endif()

# QuIK headers are header-only except geometry.cpp implementation
set(QUIK_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../quik")

add_library(quik_geometry OBJECT
    ${QUIK_SRC_DIR}/src/geometry.cpp)

target_include_directories(quik_geometry PUBLIC
    ${QUIK_SRC_DIR}/include
    $<$<BOOL:${EIGEN3_INCLUDE_DIR}>:${EIGEN3_INCLUDE_DIR}>
)

if(HAVE_EIGEN3_PACKAGE)
  target_link_libraries(quik_geometry PUBLIC Eigen3::Eigen)
endif()

pybind11_add_module(pyquik MODULE bindings.cpp $<TARGET_OBJECTS:quik_geometry>)

target_include_directories(pyquik PRIVATE
    ${QUIK_SRC_DIR}/include
    $<$<BOOL:${EIGEN3_INCLUDE_DIR}>:${EIGEN3_INCLUDE_DIR}>
)

if(HAVE_EIGEN3_PACKAGE)
  target_link_libraries(pyquik PRIVATE Eigen3::Eigen)
endif()

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