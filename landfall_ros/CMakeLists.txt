cmake_minimum_required(VERSION 3.22)
project(landfall_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosbag2_storage REQUIRED)

# Header-only compatibility layer, linked by the nodes and the CLI.
add_library(landfall_ros_compat INTERFACE)
target_include_directories(landfall_ros_compat INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
landfall_set_cxx_standard(landfall_ros_compat)
landfall_link_package(landfall_ros_compat INTERFACE cv_bridge rosbag2_storage)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS landfall_ros_compat EXPORT export_landfall_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core)
  find_package(sensor_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(bondcpp REQUIRED)
  find_package(diagnostic_updater REQUIRED)

  ament_add_gtest(test_compat_cv_bridge test/test_compat_cv_bridge.cpp)
  target_link_libraries(test_compat_cv_bridge landfall_ros_compat opencv_core)
  landfall_link_package(test_compat_cv_bridge PLAIN sensor_msgs)
  landfall_set_warnings(test_compat_cv_bridge)

  ament_add_gtest(test_compat_rosbag2 test/test_compat_rosbag2.cpp)
  target_link_libraries(test_compat_rosbag2 landfall_ros_compat)
  landfall_link_package(test_compat_rosbag2 PLAIN rclcpp rosbag2_cpp std_msgs)
  landfall_set_warnings(test_compat_rosbag2)

  ament_add_gtest(test_compat_lifecycle test/test_compat_lifecycle.cpp)
  landfall_link_package(test_compat_lifecycle PLAIN
    rclcpp rclcpp_lifecycle lifecycle_msgs bondcpp diagnostic_updater)
  landfall_set_cxx_standard(test_compat_lifecycle)
  landfall_set_warnings(test_compat_lifecycle)
endif()

ament_export_targets(export_landfall_ros)
ament_export_dependencies(landfall_core cv_bridge rosbag2_storage)
ament_package()
