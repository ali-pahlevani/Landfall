cmake_minimum_required(VERSION 3.22)
project(landfall_bringup NONE)

find_package(ament_cmake REQUIRED)

# Launch files, parameters, RViz config and simulation wrappers arrive in M6 (docs/TODO.md T-001).

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
