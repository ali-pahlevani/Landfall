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

# Provides the target `landfall_poselib`: PoseLib (BSD-3-Clause), linked privately into landfall_core.
#
# By default the pinned release tarball is downloaded (hash-checked) and compiled as a static,
# position-independent library with hidden symbols, so it is neither installed nor exported and
# can't clash with another PoseLib in the same process. PoseLib's own CMake project isn't used: it
# installs and exports itself and forces a build type.
#
# Offline builds: -DFETCHCONTENT_SOURCE_DIR_LANDFALL_POSELIB=<unpacked PoseLib-2.0.5> uses a local
# copy, and -DLANDFALL_USE_SYSTEM_POSELIB=ON uses an installed PoseLib (e.g. Ubuntu's libposelib-dev).

include_guard(GLOBAL)

option(LANDFALL_USE_SYSTEM_POSELIB "Use an installed PoseLib instead of the pinned download" OFF)
set(LANDFALL_POSELIB_VERSION 2.0.5)
set(LANDFALL_POSELIB_URL
  "https://github.com/PoseLib/PoseLib/archive/refs/tags/v${LANDFALL_POSELIB_VERSION}.tar.gz")
set(LANDFALL_POSELIB_SHA256 a9493e6725c58e6ae541fe416c0a6179185a60006880ff3ddf32737a43695668)

if(LANDFALL_USE_SYSTEM_POSELIB)
  find_package(PoseLib REQUIRED)
  add_library(landfall_poselib INTERFACE)
  target_link_libraries(landfall_poselib INTERFACE PoseLib::PoseLib)
  message(STATUS "landfall_core: using system PoseLib ${PoseLib_VERSION}")
  return()
endif()

include(FetchContent)
# Extracted files get the extraction time as timestamp (avoids a CMake >= 3.24 policy warning).
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
# SOURCE_SUBDIR points to a directory without a CMakeLists.txt, so FetchContent only downloads and
# unpacks; the library is defined below.
fetchcontent_declare(landfall_poselib
  URL "${LANDFALL_POSELIB_URL}"
  URL_HASH SHA256=${LANDFALL_POSELIB_SHA256}
  SOURCE_SUBDIR landfall-no-cmake
)
fetchcontent_makeavailable(landfall_poselib)

file(GLOB_RECURSE _landfall_poselib_sources CONFIGURE_DEPENDS
  "${landfall_poselib_SOURCE_DIR}/PoseLib/*.cc")
add_library(landfall_poselib STATIC ${_landfall_poselib_sources})
target_include_directories(landfall_poselib SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${landfall_poselib_SOURCE_DIR}>")
target_link_libraries(landfall_poselib PUBLIC Eigen3::Eigen)
landfall_set_cxx_standard(landfall_poselib)
set_target_properties(landfall_poselib PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
)
# Third-party code: upstream builds it with -Werror on its own CI; keep our logs free of its notes.
target_compile_options(landfall_poselib PRIVATE -w)
message(STATUS "landfall_core: building PoseLib ${LANDFALL_POSELIB_VERSION} from ${LANDFALL_POSELIB_URL}")
