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

# ONNX Runtime for the learned backends (docs/PLAN.md §8, M5).
#
#   LANDFALL_WITH_ONNXRUNTIME  AUTO (default): build the learned backends when ONNX Runtime is
#                              found; ON: fail without it; OFF: classical backends only.
#   ONNXRUNTIME_ROOT           (cache variable or environment) an ONNX Runtime install with
#                              include/ and lib/: an official release archive, a custom build
#                              (e.g. with CUDA), or /usr for the system's. Searched first.
#
# Then onnxruntime_vendor (rosdep: the official CPU release for x86_64 and aarch64), then the
# system (Ubuntu 26.04's libonnxruntime-dev). The release archive's own CMake config is not used:
# it points at lib64/ and include/onnxruntime/, which the archive doesn't have.
#
# Sets LANDFALL_ONNXRUNTIME_TARGET (the target to link privately), LANDFALL_ONNXRUNTIME_SOURCE and
# LANDFALL_ONNXRUNTIME_API_VERSION, or leaves LANDFALL_ONNXRUNTIME_TARGET empty.

set(LANDFALL_WITH_ONNXRUNTIME AUTO CACHE STRING "Build the learned backends: AUTO, ON or OFF")
set_property(CACHE LANDFALL_WITH_ONNXRUNTIME PROPERTY STRINGS AUTO ON OFF)
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
set(ONNXRUNTIME_ROOT "${ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime install to use first")

# The oldest ONNX Runtime the backends are written for (its C API version, = the minor version).
set(_landfall_ort_min_api 20)

set(LANDFALL_ONNXRUNTIME_TARGET "")
set(LANDFALL_ONNXRUNTIME_SOURCE "")
set(LANDFALL_ONNXRUNTIME_API_VERSION "")

if(NOT LANDFALL_WITH_ONNXRUNTIME STREQUAL "OFF")
  if(ONNXRUNTIME_ROOT)
    find_path(LANDFALL_ONNXRUNTIME_ROOT_INCLUDE onnxruntime_cxx_api.h
      PATHS "${ONNXRUNTIME_ROOT}/include" "${ONNXRUNTIME_ROOT}/include/onnxruntime"
      NO_DEFAULT_PATH)
    find_library(LANDFALL_ONNXRUNTIME_ROOT_LIBRARY onnxruntime
      PATHS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
      "${ONNXRUNTIME_ROOT}/lib/${CMAKE_LIBRARY_ARCHITECTURE}" NO_DEFAULT_PATH)
    if(LANDFALL_ONNXRUNTIME_ROOT_INCLUDE AND LANDFALL_ONNXRUNTIME_ROOT_LIBRARY)
      get_filename_component(_landfall_ort_libdir "${LANDFALL_ONNXRUNTIME_ROOT_LIBRARY}" DIRECTORY)
      add_library(landfall_onnxruntime_root SHARED IMPORTED)
      set_target_properties(landfall_onnxruntime_root PROPERTIES
        IMPORTED_LOCATION "${LANDFALL_ONNXRUNTIME_ROOT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${LANDFALL_ONNXRUNTIME_ROOT_INCLUDE}"
        INTERFACE_LINK_OPTIONS "LINKER:-rpath,${_landfall_ort_libdir}")
      set(LANDFALL_ONNXRUNTIME_TARGET landfall_onnxruntime_root)
      set(LANDFALL_ONNXRUNTIME_SOURCE "ONNXRUNTIME_ROOT ${ONNXRUNTIME_ROOT}")
    else()
      message(WARNING "ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT} has no ONNX Runtime; looking elsewhere")
    endif()
  endif()
  if(NOT LANDFALL_ONNXRUNTIME_TARGET)
    find_package(onnxruntime_vendor QUIET)
    if(onnxruntime_vendor_FOUND AND TARGET onnxruntime::onnxruntime)
      set(LANDFALL_ONNXRUNTIME_TARGET onnxruntime::onnxruntime)
      set(LANDFALL_ONNXRUNTIME_SOURCE "onnxruntime_vendor")
    endif()
  endif()
  if(NOT LANDFALL_ONNXRUNTIME_TARGET)
    find_package(onnxruntime CONFIG QUIET)
    if(onnxruntime_FOUND AND TARGET onnxruntime::onnxruntime)
      set(LANDFALL_ONNXRUNTIME_TARGET onnxruntime::onnxruntime)
      set(LANDFALL_ONNXRUNTIME_SOURCE "system (${onnxruntime_DIR})")
    endif()
  endif()

  # The C API version from the headers the target brings.
  if(LANDFALL_ONNXRUNTIME_TARGET)
    get_target_property(_landfall_ort_includes ${LANDFALL_ONNXRUNTIME_TARGET}
      INTERFACE_INCLUDE_DIRECTORIES)
    find_file(LANDFALL_ONNXRUNTIME_C_API onnxruntime_c_api.h
      PATHS ${_landfall_ort_includes} NO_DEFAULT_PATH NO_CACHE)
    if(LANDFALL_ONNXRUNTIME_C_API)
      file(STRINGS "${LANDFALL_ONNXRUNTIME_C_API}" _landfall_ort_define
        REGEX "^#define ORT_API_VERSION [0-9]+")
      string(REGEX REPLACE "^#define ORT_API_VERSION ([0-9]+).*" "\\1"
        LANDFALL_ONNXRUNTIME_API_VERSION "${_landfall_ort_define}")
    endif()
    if(NOT LANDFALL_ONNXRUNTIME_API_VERSION
      OR LANDFALL_ONNXRUNTIME_API_VERSION LESS _landfall_ort_min_api)
      message(WARNING "ONNX Runtime from ${LANDFALL_ONNXRUNTIME_SOURCE} has C API version "
        "'${LANDFALL_ONNXRUNTIME_API_VERSION}', older than 1.${_landfall_ort_min_api}: not used")
      set(LANDFALL_ONNXRUNTIME_TARGET "")
    endif()
  endif()

  if(LANDFALL_ONNXRUNTIME_TARGET)
    message(STATUS "Learned backends: ONNX Runtime 1.${LANDFALL_ONNXRUNTIME_API_VERSION} from "
      "${LANDFALL_ONNXRUNTIME_SOURCE}")
  elseif(LANDFALL_WITH_ONNXRUNTIME STREQUAL "ON")
    message(FATAL_ERROR "LANDFALL_WITH_ONNXRUNTIME=ON, but no ONNX Runtime 1.${_landfall_ort_min_api}"
      " or newer was found (ONNXRUNTIME_ROOT, onnxruntime_vendor, system)")
  else()
    message(STATUS "Learned backends: off (no ONNX Runtime found)")
  endif()
else()
  message(STATUS "Learned backends: off (LANDFALL_WITH_ONNXRUNTIME=OFF)")
endif()
