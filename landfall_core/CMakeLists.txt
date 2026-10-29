cmake_minimum_required(VERSION 3.22)
project(landfall_core LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
include(cmake/LandfallHelpers.cmake)
find_package(Ceres REQUIRED)

# Version comes from package.xml.
ament_package_xml()
string(REPLACE "." ";" _landfall_version_parts "${landfall_core_VERSION}")
list(GET _landfall_version_parts 0 landfall_core_VERSION_MAJOR)
list(GET _landfall_version_parts 1 landfall_core_VERSION_MINOR)
list(GET _landfall_version_parts 2 landfall_core_VERSION_PATCH)
set(_landfall_generated_include "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
configure_file(include/landfall/version.hpp.in
  "${_landfall_generated_include}/landfall/version.hpp" @ONLY)

add_library(landfall_core SHARED
  src/version.cpp
)
target_include_directories(landfall_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${_landfall_generated_include}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
landfall_set_cxx_standard(landfall_core)
landfall_set_warnings(landfall_core)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME} PATTERN "*.in" EXCLUDE)
install(FILES "${_landfall_generated_include}/landfall/version.hpp"
  DESTINATION include/${PROJECT_NAME}/landfall)
install(TARGETS landfall_core EXPORT export_landfall_core
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

  ament_add_gtest(test_version test/test_version.cpp)
  target_link_libraries(test_version landfall_core)
  landfall_set_warnings(test_version)

  ament_add_gtest(test_compat_ceres test/test_compat_ceres.cpp)
  target_link_libraries(test_compat_ceres landfall_core Ceres::ceres)
  landfall_set_warnings(test_compat_ceres)
endif()

ament_export_targets(export_landfall_core HAS_LIBRARY_TARGET)
ament_package(CONFIG_EXTRAS cmake/landfall_core-extras.cmake)
