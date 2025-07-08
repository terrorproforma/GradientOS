cmake_minimum_required(VERSION 3.16)
project(spherical_wrist_solver LANGUAGES CXX)

# Enforce C++17 for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# User-options ---------------------------------------------------------------
# ---------------------------------------------------------------------------
set(URDF_FILE "mini-6dof-arm/spherical-mini-6dof-arm.urdf" CACHE FILEPATH "URDF file used to generate ik_poe.hpp")
set(TOOL_LINK  "tool_link"                             CACHE STRING   "Name of the tool link (end-effector frame) in the URDF")

# ---------------------------------------------------------------------------
# Dependencies ---------------------------------------------------------------
# ---------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
include(FetchContent)

# ----------------------------------------------------------
# 1. Eigen3 (header-only) – use system package if available,
#    otherwise fetch v3.4.0 from GitLab
# ----------------------------------------------------------
find_package(Eigen3 CONFIG QUIET)

if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found – fetching 3.4.0 via FetchContent")
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    # FetchContent exports Eigen3::Eigen target for us
endif()

# ---------------------------------------------------------------------------
# Generate ik_poe.hpp (build-time) ------------------------------------------
# ---------------------------------------------------------------------------
set(GENERATED_DIR ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${GENERATED_DIR})
set(POE_HEADER ${GENERATED_DIR}/ik_poe.hpp)

add_custom_command(
    OUTPUT  ${POE_HEADER}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/generate_poe_header.py
            ${CMAKE_SOURCE_DIR}/${URDF_FILE} ${TOOL_LINK}
            > ${POE_HEADER}
    DEPENDS tools/generate_poe_header.py ${URDF_FILE}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Generating ik_poe.hpp from ${URDF_FILE}"
    VERBATIM)

add_custom_target(generate_poe_header DEPENDS ${POE_HEADER})

# ---------------------------------------------------------------------------
# Header-only library --------------------------------------------------------
# ---------------------------------------------------------------------------
add_library(spherical_ik INTERFACE)
add_dependencies(spherical_ik generate_poe_header)

target_include_directories(spherical_ik
    INTERFACE
        $<BUILD_INTERFACE:${GENERATED_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(spherical_ik
    INTERFACE Eigen3::Eigen)

target_compile_features(spherical_ik INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# Optional: build Python bindings via pybind11 ------------------------------
# ---------------------------------------------------------------------------
# ----------------------------------------------------------
# 2. pybind11 – build Python bindings; fetch if necessary
# ----------------------------------------------------------
find_package(pybind11 CONFIG QUIET)

if(NOT pybind11_FOUND)
    message(STATUS "pybind11 not found – fetching v2.11.1 via FetchContent")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.11.1
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(pybind11)
endif()

if(EXISTS ${CMAKE_SOURCE_DIR}/spherical_ik_py.cpp)
    pybind11_add_module(spherical_ik_py MODULE spherical_ik_py.cpp)

    target_include_directories(spherical_ik_py PRIVATE
        ${GENERATED_DIR}
        ${CMAKE_SOURCE_DIR}/include)

    target_link_libraries(spherical_ik_py PRIVATE Eigen3::Eigen)
    target_compile_features(spherical_ik_py PRIVATE cxx_std_17)

    add_custom_command(TARGET spherical_ik_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:spherical_ik_py>
                ${CMAKE_SOURCE_DIR}            # copies into src/spherical_wrist_solver/
        COMMENT "Copying Python module for easy import")
endif()

# ---------------------------------------------------------------------------
# Example test executable ----------------------------------------------------
# ---------------------------------------------------------------------------
if (EXISTS ${CMAKE_SOURCE_DIR}/src/test.cpp)
    add_executable(test_solver src/test.cpp)
    target_link_libraries(test_solver PRIVATE spherical_ik)
endif() 