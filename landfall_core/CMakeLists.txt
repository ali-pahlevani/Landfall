cmake_minimum_required(VERSION 3.22)
project(landfall_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(cmake/LandfallHelpers.cmake)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(SQLite3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
include(cmake/PoseLib.cmake)

# yaml-cpp 0.7 (Humble) exports `yaml-cpp`, 0.8 exports `yaml-cpp::yaml-cpp`.
if(TARGET yaml-cpp::yaml-cpp)
  set(_landfall_yaml_cpp yaml-cpp::yaml-cpp)
else()
  set(_landfall_yaml_cpp yaml-cpp)
endif()

# Version comes from package.xml.
ament_package_xml()
string(REPLACE "." ";" _landfall_version_parts "${landfall_core_VERSION}")
list(GET _landfall_version_parts 0 landfall_core_VERSION_MAJOR)
list(GET _landfall_version_parts 1 landfall_core_VERSION_MINOR)
list(GET _landfall_version_parts 2 landfall_core_VERSION_PATCH)
set(_landfall_generated_include "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
configure_file(include/landfall/version.hpp.in
  "${_landfall_generated_include}/landfall/version.hpp" @ONLY)

# Runtime library: everything the online relocalizer needs. Doesn't link Ceres.
add_library(landfall_core SHARED
  src/camera_model.cpp
  src/db/database.cpp
  src/db/migrations.cpp
  src/features/matchers.cpp
  src/features/opencv_features.cpp
  src/features/retrieval.cpp
  src/features/vocab_tree.cpp
  src/geometry/gravity.cpp
  src/geometry/pnp.cpp
  src/query/clustering.cpp
  src/query/decision.cpp
  src/query/extrinsics_monitor.cpp
  src/query/sanity.cpp
  src/grid/likelihood_field.cpp
  src/grid/map_loader.cpp
  src/grid/occupancy_grid.cpp
  src/grid/scan_verifier.cpp
  src/types.cpp
  src/util/sha256.cpp
  src/version.cpp
)
target_include_directories(landfall_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${_landfall_generated_include}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
# OpenCV modules are listed explicitly: no videoio/highgui (they pull in ffmpeg/GTK).
target_link_libraries(landfall_core
  PUBLIC
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_features2d
    opencv_calib3d
  PRIVATE
    opencv_imgcodecs
    SQLite::SQLite3
    ${_landfall_yaml_cpp}
    landfall_poselib
)
landfall_set_cxx_standard(landfall_core)
landfall_set_warnings(landfall_core)

# Offline map-building library: triangulation and bundle adjustment. Only it links Ceres (and,
# through Ubuntu's Ceres, SuiteSparse), so the online relocalizer stays free of both.
add_library(landfall_core_build SHARED
  src/build/triangulation.cpp
)
target_link_libraries(landfall_core_build PUBLIC landfall_core PRIVATE Ceres::ceres)
landfall_set_cxx_standard(landfall_core_build)
landfall_set_warnings(landfall_core_build)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} PATTERN "*.in" EXCLUDE)
install(FILES "${_landfall_generated_include}/landfall/version.hpp"
  DESTINATION include/${PROJECT_NAME}/landfall)
install(TARGETS landfall_core landfall_core_build EXPORT export_landfall_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES cmake/LandfallHelpers.cmake DESTINATION share/${PROJECT_NAME}/cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Formatting is enforced by clang-format/clang-tidy in a pinned lint job, not uncrustify/cpplint.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # landfall_core_test(<name> [TIMEOUT <s>] [LIBRARIES <targets>...]): test/<name>.cpp
  function(landfall_core_test name)
    cmake_parse_arguments(ARG "" "TIMEOUT" "LIBRARIES" ${ARGN})
    if(NOT ARG_TIMEOUT)
      set(ARG_TIMEOUT 60)
    endif()
    ament_add_gtest(${name} test/${name}.cpp TIMEOUT ${ARG_TIMEOUT})
    target_link_libraries(${name} landfall_core ${ARG_LIBRARIES})
    landfall_set_warnings(${name})
  endfunction()

  landfall_core_test(test_version)
  landfall_core_test(test_compat_ceres LIBRARIES Ceres::ceres)
  landfall_core_test(test_types)
  landfall_core_test(test_camera_model)
  landfall_core_test(test_sha256)
  landfall_core_test(test_database LIBRARIES SQLite::SQLite3)
  landfall_core_test(test_map_loader LIBRARIES opencv_imgcodecs)
  landfall_core_test(test_scan_verifier)
  landfall_core_test(test_features TIMEOUT 300)
  landfall_core_test(test_vocab_tree TIMEOUT 300)
  landfall_core_test(test_pnp TIMEOUT 600)
  landfall_core_test(test_triangulation TIMEOUT 300 LIBRARIES landfall_core_build)
  landfall_core_test(test_query)
endif()

ament_export_targets(export_landfall_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)
ament_package(CONFIG_EXTRAS cmake/landfall_core-extras.cmake)
