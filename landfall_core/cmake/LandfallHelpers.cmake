# Copyright 2026 Ali Pahlevani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# CMake helpers shared by all Landfall packages (installed with landfall_core and
# loaded automatically by find_package(landfall_core)).

include_guard(GLOBAL)

option(LANDFALL_WERROR "Treat compiler warnings in Landfall targets as errors" OFF)

# Lyrical and later ship ament_cmake_ros_core, whose ament_ros_defaults target requires C++20
# for everything linking rclcpp. Non-ROS Landfall targets follow the same standard so the whole
# workspace compiles as one language level: C++17 on Humble/Jazzy, C++20 on Lyrical+.
find_package(ament_cmake_ros_core QUIET)
if(TARGET ament_cmake_ros_core::ament_ros_cxx_standard)
  get_target_property(LANDFALL_CXX_FEATURE
    ament_cmake_ros_core::ament_ros_cxx_standard INTERFACE_COMPILE_FEATURES)
elseif(TARGET ament_cmake_ros_core::ament_ros_defaults)
  set(LANDFALL_CXX_FEATURE cxx_std_20)
else()
  set(LANDFALL_CXX_FEATURE cxx_std_17)
endif()

# Keyword used to attach usage requirements: INTERFACE libraries accept nothing else.
function(_landfall_usage_keyword target out_var)
  get_target_property(_type ${target} TYPE)
  if(_type STREQUAL "INTERFACE_LIBRARY")
    set(${out_var} INTERFACE PARENT_SCOPE)
  else()
    set(${out_var} PUBLIC PARENT_SCOPE)
  endif()
endfunction()

# landfall_set_cxx_standard(<target>)
function(landfall_set_cxx_standard target)
  _landfall_usage_keyword(${target} _kw)
  target_compile_features(${target} ${_kw} ${LANDFALL_CXX_FEATURE})
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endfunction()

# landfall_set_warnings(<target>)
function(landfall_set_warnings target)
  get_target_property(_type ${target} TYPE)
  if(_type STREQUAL "INTERFACE_LIBRARY")
    return()
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(LANDFALL_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

# landfall_link_package(<target> <PUBLIC|PRIVATE|INTERFACE|PLAIN> <package>...)
#
# Target-based linking without ament_target_dependencies. Uses the package's exported CMake
# targets when it has them; packages that export none (Nav2 on Humble and Jazzy) fall back to
# their legacy <pkg>_INCLUDE_DIRS / _LIBRARIES / _DEFINITIONS variables. Call find_package()
# for each package first.
#
# PLAIN links with the keyword-less target_link_libraries() signature. Use it for targets made by
# ament_add_gtest(), which links that way itself; CMake forbids mixing the two signatures.
function(landfall_link_package target visibility)
  if(visibility STREQUAL "PLAIN")
    set(_link_kw "")
    set(_usage_kw PRIVATE)
  else()
    set(_link_kw ${visibility})
    set(_usage_kw ${visibility})
  endif()
  foreach(pkg IN LISTS ARGN)
    if(NOT ${pkg}_FOUND)
      message(FATAL_ERROR "landfall_link_package: find_package(${pkg}) must be called first")
    endif()
    if(${pkg}_TARGETS)
      target_link_libraries(${target} ${_link_kw} ${${pkg}_TARGETS})
    elseif(TARGET ${pkg}::${pkg})
      target_link_libraries(${target} ${_link_kw} ${pkg}::${pkg})
    else()
      if(${pkg}_INCLUDE_DIRS)
        target_include_directories(${target} SYSTEM ${_usage_kw} ${${pkg}_INCLUDE_DIRS})
      endif()
      if(${pkg}_LIBRARIES)
        target_link_libraries(${target} ${_link_kw} ${${pkg}_LIBRARIES})
      endif()
      if(${pkg}_DEFINITIONS)
        target_compile_definitions(${target} ${_usage_kw} ${${pkg}_DEFINITIONS})
      endif()
    endif()
  endforeach()
endfunction()
