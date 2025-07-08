cmake_minimum_required(VERSION 3.14)

project(ik_solver LANGUAGES CXX)

# ----------------------------------------------------------------------------
# User options
# ----------------------------------------------------------------------------
option(BUILD_PYTHON_MODULE "Build the Python extension module with pybind11" ON)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
# Eigen: header-only linear-algebra library ----------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# pybind11: header-only binding generator ------------------------------------
if(BUILD_PYTHON_MODULE)
  find_package(pybind11 2.11 REQUIRED)
endif()

# ----------------------------------------------------------------------------
# Core library  (pure C++, no Python)
# ----------------------------------------------------------------------------
add_library(ik_core STATIC
  solver.cpp
)

target_include_directories(ik_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(ik_core PUBLIC Eigen3::Eigen)

# ----------------------------------------------------------------------------
# Python extension module (optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_MODULE)
  pybind11_add_module(ik_solver MODULE
    bindings.cpp
  )
  target_link_libraries(ik_solver PRIVATE ik_core)
  target_include_directories(ik_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

# ----------------------------------------------------------------------------
# Installation (optional)
# ----------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS ik_core
        EXPORT ik_core_targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(BUILD_PYTHON_MODULE)
  install(TARGETS ik_solver LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

install(FILES solver.hpp math.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}) 

# ----------------------------------------------------------------------------
# Optional: auto-generate poe.hpp from a URDF via generate_poe_header.py
# ----------------------------------------------------------------------------
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(IK_URDF_PATH "" CACHE STRING "Path to the robot URDF file (leave empty to skip auto-generation)")
set(IK_TOOL_LINK "" CACHE STRING "Name of the tool link inside the URDF (optional)")

if(IK_URDF_PATH)
  if(NOT EXISTS "${IK_URDF_PATH}")
    message(FATAL_ERROR "IK_URDF_PATH='${IK_URDF_PATH}' does not exist")
  endif()
  set(POE_HPP ${CMAKE_CURRENT_SOURCE_DIR}/poe.hpp)
  if(IK_TOOL_LINK)
    add_custom_command(
      OUTPUT ${POE_HPP}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_poe_header.py ${IK_URDF_PATH} ${IK_TOOL_LINK} > ${POE_HPP}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_poe_header.py ${IK_URDF_PATH}
      COMMENT "Generating poe.hpp from ${IK_URDF_PATH} (tool link: ${IK_TOOL_LINK})"
      VERBATIM)
  else()
    add_custom_command(
      OUTPUT ${POE_HPP}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_poe_header.py ${IK_URDF_PATH} > ${POE_HPP}
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_poe_header.py ${IK_URDF_PATH}
      COMMENT "Generating poe.hpp from ${IK_URDF_PATH}"
      VERBATIM)
  endif()
  add_custom_target(generate_poe DEPENDS ${POE_HPP})
  add_dependencies(ik_core generate_poe)
  if(BUILD_PYTHON_MODULE)
    add_dependencies(ik_solver generate_poe)
  endif()
endif() 