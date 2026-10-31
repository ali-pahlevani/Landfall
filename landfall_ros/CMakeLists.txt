cmake_minimum_required(VERSION 3.22)
project(landfall_ros LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)
find_package(landfall_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(bondcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# The CLI defaults to the map_server pixel rules of the distro it was built for.
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(LANDFALL_DEFAULT_MAP_SEMANTICS humble)
else()
  set(LANDFALL_DEFAULT_MAP_SEMANTICS jazzy)
endif()

# Header-only compatibility layer, linked by the nodes and the CLI.
add_library(landfall_ros_compat INTERFACE)
target_include_directories(landfall_ros_compat INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
landfall_set_cxx_standard(landfall_ros_compat)
landfall_link_package(landfall_ros_compat INTERFACE cv_bridge rosbag2_storage)

# ROS <-> core glue shared by the recorder and the CLI: conversions, TF pose source, IMU gravity,
# the keyframe gate and the bag reader.
add_library(landfall_ros_common SHARED
  src/bag_reader.cpp
  src/conversions.cpp
  src/frame_gate.cpp
  src/imu_gravity_source.cpp
  src/tf_pose_source.cpp
)
target_link_libraries(landfall_ros_common
  PUBLIC landfall_ros_compat landfall_core::landfall_core landfall_core::landfall_core_build
  PRIVATE opencv_imgcodecs
)
landfall_link_package(landfall_ros_common PUBLIC
  builtin_interfaces geometry_msgs rclcpp rosbag2_cpp rosbag2_storage sensor_msgs tf2 tf2_msgs)
landfall_set_cxx_standard(landfall_ros_common)
landfall_set_warnings(landfall_ros_common)

# Keyframe recorder: lifecycle node, also a component.
add_library(landfall_recorder SHARED src/recorder_node.cpp)
target_link_libraries(landfall_recorder PUBLIC landfall_ros_common)
landfall_link_package(landfall_recorder PUBLIC
  bondcpp diagnostic_msgs diagnostic_updater landfall_msgs lifecycle_msgs rclcpp_components
  rclcpp_lifecycle std_srvs tf2_ros visualization_msgs)
landfall_set_cxx_standard(landfall_recorder)
landfall_set_warnings(landfall_recorder)
rclcpp_components_register_nodes(landfall_recorder "landfall::ros::RecorderNode")

add_executable(recorder_node src/recorder_main.cpp)
target_link_libraries(recorder_node landfall_recorder)
landfall_set_cxx_standard(recorder_node)
landfall_set_warnings(recorder_node)

# `landfall` CLI: build, inspect, report, verify, migrate.
add_executable(landfall src/cli/landfall_cli.cpp)
target_link_libraries(landfall landfall_ros_common opencv_imgcodecs)
target_compile_definitions(landfall PRIVATE
  LANDFALL_DEFAULT_MAP_SEMANTICS="${LANDFALL_DEFAULT_MAP_SEMANTICS}")
landfall_set_cxx_standard(landfall)
landfall_set_warnings(landfall)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS landfall_ros_compat landfall_ros_common landfall_recorder
  EXPORT export_landfall_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS recorder_node landfall DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)

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

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_link_libraries(test_conversions landfall_ros_common opencv_imgcodecs)
  landfall_set_warnings(test_conversions)

  ament_add_gtest(test_bag_build test/test_bag_build.cpp TIMEOUT 300)
  target_link_libraries(test_bag_build landfall_ros_common)
  target_compile_definitions(test_bag_build PRIVATE LANDFALL_CLI="$<TARGET_FILE:landfall>")
  add_dependencies(test_bag_build landfall)
  landfall_set_warnings(test_bag_build)

  # The depot fixture through the offline pipeline (test/fixtures/depot_small/README.md).
  ament_add_gtest(test_fixture_build test/test_fixture_build.cpp TIMEOUT 300)
  target_link_libraries(test_fixture_build landfall_ros_common)
  target_compile_definitions(test_fixture_build PRIVATE
    LANDFALL_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/depot_small"
    LANDFALL_ROS_DISTRO="$ENV{ROS_DISTRO}")
  landfall_set_warnings(test_fixture_build)

  ament_add_gtest(test_recorder_node test/test_recorder_node.cpp TIMEOUT 120)
  target_link_libraries(test_recorder_node landfall_recorder)
  landfall_set_warnings(test_recorder_node)

  # The map loader against the real nav2_map_server of this distro (launch test).
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  add_executable(map_parity_check test/map_parity_check.cpp)
  target_link_libraries(map_parity_check PRIVATE landfall_core::landfall_core)
  landfall_link_package(map_parity_check PRIVATE nav_msgs rclcpp)
  target_compile_definitions(map_parity_check PRIVATE LANDFALL_ROS_DISTRO="$ENV{ROS_DISTRO}")
  landfall_set_cxx_standard(map_parity_check)
  landfall_set_warnings(map_parity_check)
  add_launch_test(test/test_map_server_parity.py
    TIMEOUT 180
    ENV LANDFALL_MAP_PARITY_CHECK=$<TARGET_FILE:map_parity_check>)
endif()

ament_export_targets(export_landfall_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(
  landfall_core landfall_msgs OpenCV bondcpp builtin_interfaces cv_bridge diagnostic_msgs
  diagnostic_updater geometry_msgs lifecycle_msgs rclcpp rclcpp_components rclcpp_lifecycle
  rosbag2_cpp rosbag2_storage sensor_msgs std_srvs tf2 tf2_msgs tf2_ros visualization_msgs)
ament_package()
