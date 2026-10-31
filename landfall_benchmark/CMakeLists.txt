cmake_minimum_required(VERSION 3.22)
project(landfall_benchmark LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)
find_package(landfall_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Evaluation against a simulator's ground truth. The kidnap benchmark runner and scenarios arrive
# in M7 (docs/TODO.md T-002).
add_executable(evaluate_poses src/evaluate_poses.cpp)
target_link_libraries(evaluate_poses PRIVATE
  landfall_core::landfall_core landfall_ros::landfall_ros_common)
landfall_link_package(evaluate_poses PRIVATE geometry_msgs rclcpp rosbag2_cpp rosbag2_storage)
landfall_set_cxx_standard(evaluate_poses)
landfall_set_warnings(evaluate_poses)

install(TARGETS evaluate_poses DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
