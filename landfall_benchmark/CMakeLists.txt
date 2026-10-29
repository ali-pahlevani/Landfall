cmake_minimum_required(VERSION 3.22)
project(landfall_benchmark NONE)

find_package(ament_cmake REQUIRED)

# The kidnap benchmark runner and scenarios arrive in M7 (docs/TODO.md T-002).

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
