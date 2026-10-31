cmake_minimum_required(VERSION 3.22)
project(landfall_bringup NONE)

find_package(ament_cmake REQUIRED)

# Simulation assets (docs/recording.md): a TurtleBot 4 description with a 640 x 480 camera and
# ground-truth pose. They need the simulation image (docker/Dockerfile.sim), so the simulator
# packages are not dependencies of this package. Launch files, parameters and RViz config arrive
# in M6 (docs/TODO.md T-001).
install(DIRECTORY sim DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
