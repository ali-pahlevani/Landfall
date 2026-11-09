cmake_minimum_required(VERSION 3.22)
project(landfall_benchmark LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)
find_package(landfall_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(landfall_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Placement sampling and the benchmark's metrics (docs/benchmark.md), for the tools and the tests.
add_library(landfall_benchmark_lib STATIC src/placements.cpp src/metrics.cpp)
target_include_directories(landfall_benchmark_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(landfall_benchmark_lib PUBLIC landfall_core::landfall_core)
set_target_properties(landfall_benchmark_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
landfall_set_cxx_standard(landfall_benchmark_lib)
landfall_set_warnings(landfall_benchmark_lib)

# Evaluation against a simulator's ground truth: recorded poses, the scan check.
add_executable(evaluate_poses src/evaluate_poses.cpp)
target_link_libraries(evaluate_poses PRIVATE
  landfall_core::landfall_core landfall_ros::landfall_ros_common)
landfall_link_package(evaluate_poses PRIVATE
  geometry_msgs nav_msgs rclcpp rosbag2_cpp rosbag2_storage tf2_msgs)
landfall_set_cxx_standard(evaluate_poses)
landfall_set_warnings(evaluate_poses)

add_executable(evaluate_scan_check src/evaluate_scan_check.cpp)
target_link_libraries(evaluate_scan_check PRIVATE
  landfall_core::landfall_core landfall_ros::landfall_ros_common)
landfall_set_cxx_standard(evaluate_scan_check)
landfall_set_warnings(evaluate_scan_check)

# The kidnap benchmark (plan §11 M7): the runner in the simulation, and its results tables.
add_executable(kidnap_runner src/kidnap_runner.cpp)
target_link_libraries(kidnap_runner PRIVATE
  landfall_benchmark_lib landfall_ros::landfall_ros_common)
landfall_link_package(kidnap_runner PRIVATE
  geometry_msgs landfall_msgs rclcpp ros_gz_interfaces std_srvs)
landfall_set_cxx_standard(kidnap_runner)
landfall_set_warnings(kidnap_runner)

add_executable(summarize_benchmark src/summarize_benchmark.cpp)
target_link_libraries(summarize_benchmark PRIVATE landfall_benchmark_lib)
landfall_set_cxx_standard(summarize_benchmark)
landfall_set_warnings(summarize_benchmark)

add_executable(sample_placements src/sample_placements.cpp)
target_link_libraries(sample_placements PRIVATE landfall_benchmark_lib)
landfall_set_cxx_standard(sample_placements)
landfall_set_warnings(sample_placements)

# The field validation (PLAN M8): the relocalizer's shadow log against the localization.
add_executable(summarize_shadow src/summarize_shadow.cpp)
target_link_libraries(summarize_shadow PRIVATE landfall_benchmark_lib)
landfall_set_cxx_standard(summarize_shadow)
landfall_set_warnings(summarize_shadow)

install(TARGETS evaluate_poses evaluate_scan_check kidnap_runner sample_placements
  summarize_benchmark summarize_shadow DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scenarios DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics test/test_metrics.cpp)
  target_link_libraries(test_metrics landfall_benchmark_lib)
  landfall_set_cxx_standard(test_metrics)
  ament_add_gtest(test_placements test/test_placements.cpp)
  target_link_libraries(test_placements landfall_benchmark_lib)
  landfall_set_cxx_standard(test_placements)
endif()

ament_package()
