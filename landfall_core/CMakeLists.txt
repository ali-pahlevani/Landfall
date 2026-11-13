cmake_minimum_required(VERSION 3.22)
project(landfall_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(cmake/LandfallHelpers.cmake)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d)
find_package(SQLite3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)
include(cmake/PoseLib.cmake)
include(cmake/LandfallOnnxRuntime.cmake)

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
  src/backend_options.cpp
  src/camera_model.cpp
  src/db/codec.cpp
  src/db/database.cpp
  src/db/migrations.cpp
  src/eval/trajectory.cpp
  src/features/floor_mask.cpp
  src/features/matchers.cpp
  src/features/opencv_features.cpp
  src/features/retrieval.cpp
  src/features/vocab_tree.cpp
  src/geometry/gravity.cpp
  src/geometry/pnp.cpp
  src/geometry/two_view.cpp
  src/query/auto_trigger.cpp
  src/query/clustering.cpp
  src/query/decision.cpp
  src/query/extrinsics_monitor.cpp
  src/query/kidnap_watchdog.cpp
  src/query/localization_map.cpp
  src/query/localizer.cpp
  src/query/sanity.cpp
  src/grid/likelihood_field.cpp
  src/grid/map_loader.cpp
  src/grid/occupancy_grid.cpp
  src/grid/scan_matcher.cpp
  src/grid/scan_verifier.cpp
  src/record/image_quality.cpp
  src/record/keyframe_selector.cpp
  src/record/staging.cpp
  src/registry.cpp
  src/types.cpp
  src/util/json.cpp
  src/util/sha256.cpp
  src/util/thread_pool.cpp
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
    Threads::Threads
    ${_landfall_yaml_cpp}
    landfall_poselib
)
landfall_set_cxx_standard(landfall_core)
landfall_set_warnings(landfall_core)

# Offline map-building library: pair selection, tracks, triangulation, bundle adjustment, the
# builder and its report. Only it links Ceres (and, through Ubuntu's Ceres, SuiteSparse), so the
# online relocalizer stays free of both.
add_library(landfall_core_build SHARED
  src/build/builder_config.cpp
  src/build/coverage.cpp
  src/build/map_builder.cpp
  src/build/pair_selection.cpp
  src/build/pose_prior_ba.cpp
  src/build/report.cpp
  src/build/tracks.cpp
  src/build/triangulation.cpp
)
target_link_libraries(landfall_core_build
  PUBLIC landfall_core
  PRIVATE Ceres::ceres opencv_imgcodecs ${_landfall_yaml_cpp})
landfall_set_cxx_standard(landfall_core_build)
landfall_set_warnings(landfall_core_build)

# Learned backends on ONNX Runtime (optional; docs/models.md). ONNX Runtime stays private: none of
# its headers is installed or needed downstream, which tests for landfall_core::landfall_core_onnx.
set(_landfall_targets landfall_core landfall_core_build)
set(_landfall_header_excludes PATTERN "*.in" EXCLUDE)
if(LANDFALL_ONNXRUNTIME_TARGET)
  add_library(landfall_core_onnx SHARED
    src/onnx/backends.cpp
    src/onnx/global_descriptor.cpp
    src/onnx/session.cpp
    src/onnx/xfeat.cpp
  )
  target_link_libraries(landfall_core_onnx
    PUBLIC landfall_core
    PRIVATE ${LANDFALL_ONNXRUNTIME_TARGET} opencv_imgproc)
  landfall_set_cxx_standard(landfall_core_onnx)
  landfall_set_warnings(landfall_core_onnx)
  list(APPEND _landfall_targets landfall_core_onnx)
else()
  list(APPEND _landfall_header_excludes PATTERN "onnx" EXCLUDE)
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} ${_landfall_header_excludes})
install(FILES "${_landfall_generated_include}/landfall/version.hpp"
  DESTINATION include/${PROJECT_NAME}/landfall)
install(TARGETS ${_landfall_targets} EXPORT export_landfall_core
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
    # Sanitizer builds run our code several times slower (test_map_builder: ~14x under ASan).
    if(CMAKE_CXX_FLAGS MATCHES "-fsanitize")
      math(EXPR ARG_TIMEOUT "${ARG_TIMEOUT} * 3")
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
  landfall_core_test(test_scan_matcher)
  landfall_core_test(test_scan_verifier)
  landfall_core_test(test_features TIMEOUT 300)
  landfall_core_test(test_vocab_tree TIMEOUT 300)
  landfall_core_test(test_pnp TIMEOUT 600)
  landfall_core_test(test_triangulation TIMEOUT 300 LIBRARIES landfall_core_build)
  landfall_core_test(test_query)
  landfall_core_test(test_thread_pool)
  landfall_core_test(test_record LIBRARIES opencv_imgcodecs)
  landfall_core_test(test_registry)
  landfall_core_test(test_two_view TIMEOUT 120)
  landfall_core_test(test_codec)
  landfall_core_test(test_tracks LIBRARIES landfall_core_build)
  landfall_core_test(test_pose_prior_ba TIMEOUT 120 LIBRARIES landfall_core_build)
  landfall_core_test(test_builder_config LIBRARIES landfall_core_build)
  landfall_core_test(test_coverage LIBRARIES landfall_core_build)
  landfall_core_test(test_map_builder TIMEOUT 900 LIBRARIES landfall_core_build)
  landfall_core_test(test_trajectory)
  landfall_core_test(test_localizer TIMEOUT 300 LIBRARIES landfall_core_build)
  landfall_core_test(test_auto_trigger)
  landfall_core_test(test_kidnap_watchdog)
  if(TARGET landfall_core_onnx)
    # The learned backends on synthetic models (test/data, landfall_tools/export/synthetic.py).
    landfall_core_test(test_onnx LIBRARIES landfall_core_onnx opencv_imgproc)
    target_compile_definitions(test_onnx PRIVATE
      LANDFALL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    # Parity with PyTorch: the synthetic models always, the real ones when they are present.
    landfall_core_test(test_onnx_parity TIMEOUT 300 LIBRARIES landfall_core_onnx)
    target_compile_definitions(test_onnx_parity PRIVATE
      LANDFALL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    target_link_libraries(test_localizer landfall_core_onnx)
    target_compile_definitions(test_localizer PRIVATE LANDFALL_TEST_ONNX
      LANDFALL_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
endif()

ament_export_targets(export_landfall_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)
ament_package(CONFIG_EXTRAS cmake/landfall_core-extras.cmake)
