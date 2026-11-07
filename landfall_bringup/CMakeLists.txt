cmake_minimum_required(VERSION 3.22)
# No compiled code, but the CMake configuration of the tests' dependencies (rclpy, the message
# packages) needs C and C++ on Humble and Lyrical.
project(landfall_bringup LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)

# Launch files and parameters for robots (docs/nav2_integration.md), and the simulation assets
# (docs/recording.md: a TurtleBot 4 description with a 640 x 480 camera and ground-truth pose).
# The simulation needs the simulation image (docker/Dockerfile.sim), and navigation.launch.xml
# needs nav2_bringup, which pulls in the simulators: neither is a dependency of this package.
install(DIRECTORY launch params sim DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  # The installed launch files, their nodes in a namespace of their own (see the tests).
  find_package(launch_testing_ament_cmake REQUIRED)
  foreach(_test test_localization_launch test_relocalizer_launch)
    add_launch_test(test/${_test}.py
      TIMEOUT 180
      ENV LANDFALL_BRINGUP_SHARE=${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
      APPEND_ENV
        AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
        PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)
  endforeach()
endif()

ament_package()
