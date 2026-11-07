cmake_minimum_required(VERSION 3.22)
project(landfall_nav2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)  # CMake helpers only; the plugin doesn't link it
find_package(landfall_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)

# Pick the BehaviorTree.CPP major version from the Nav2 release, not from whichever BT package
# happens to be installed (Humble can have both): Nav2 1.1.x (Humble) uses v3, 1.2+ uses v4.
# Nav2 1.4+ puts a nav2::LifecycleNode on the blackboard instead of an rclcpp::Node. The example
# behavior tree follows the release's own default tree (docs/PLAN.md §9).
if(nav2_behavior_tree_VERSION VERSION_LESS "1.2.0")
  set(LANDFALL_BTCPP_V4 0)
  set(_landfall_bt_package behaviortree_cpp_v3)
  set(_landfall_tree_dir humble)
elseif(nav2_behavior_tree_VERSION VERSION_LESS "1.4.0")
  set(LANDFALL_BTCPP_V4 1)
  set(_landfall_bt_package behaviortree_cpp)
  set(_landfall_tree_dir jazzy)
else()
  set(LANDFALL_BTCPP_V4 1)
  set(_landfall_bt_package behaviortree_cpp)
  set(_landfall_tree_dir lyrical)
endif()
if(nav2_behavior_tree_VERSION VERSION_LESS "1.4.0")
  set(LANDFALL_NAV2_LIFECYCLE_NODE 0)
  set(_landfall_nav2_node_package "")
else()
  set(LANDFALL_NAV2_LIFECYCLE_NODE 1)
  set(_landfall_nav2_node_package nav2_ros_common)
  find_package(nav2_ros_common REQUIRED)
endif()
find_package(${_landfall_bt_package} REQUIRED)
message(STATUS "landfall_nav2: Nav2 ${nav2_behavior_tree_VERSION}, using ${_landfall_bt_package}, "
  "behavior trees for ${_landfall_tree_dir}")

# The behavior tree plugin library: ReLocalize and IsLocalized (docs/nav2_integration.md).
add_library(landfall_nav2_bt_nodes SHARED
  src/is_localized_condition.cpp
  src/plugin.cpp
  src/relocalize_action.cpp
)
target_include_directories(landfall_nav2_bt_nodes PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_definitions(landfall_nav2_bt_nodes
  PUBLIC LANDFALL_BTCPP_V4=${LANDFALL_BTCPP_V4}
  LANDFALL_NAV2_LIFECYCLE_NODE=${LANDFALL_NAV2_LIFECYCLE_NODE}
  PRIVATE BT_PLUGIN_EXPORT)
landfall_link_package(landfall_nav2_bt_nodes PUBLIC
  ${_landfall_bt_package} geometry_msgs landfall_msgs nav2_behavior_tree rclcpp
  ${_landfall_nav2_node_package})
landfall_set_cxx_standard(landfall_nav2_bt_nodes)
landfall_set_warnings(landfall_nav2_bt_nodes)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS landfall_nav2_bt_nodes EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
# One installed name, the tree matching this Nav2 release.
install(FILES behavior_trees/${_landfall_tree_dir}/navigate_to_pose_w_relocalization.xml
  DESTINATION share/${PROJECT_NAME}/behavior_trees)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${_landfall_bt_package} geometry_msgs landfall_msgs nav2_behavior_tree rclcpp
  ${_landfall_nav2_node_package})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(tf2_ros REQUIRED)

  # Each test on a ROS domain of its own (domain_coordinator): colcon tests packages in parallel,
  # and these serve landfall_relocalizer/relocalize and Nav2's servers.
  foreach(_test test_relocalize_action test_is_localized_condition test_behavior_trees)
    ament_add_ros_isolated_gtest(${_test} test/${_test}.cpp TIMEOUT 120)
    target_link_libraries(${_test} landfall_nav2_bt_nodes)
    landfall_link_package(${_test} PLAIN nav2_msgs rclcpp_action tf2_ros)
    landfall_set_cxx_standard(${_test})
    landfall_set_warnings(${_test})
  endforeach()
  # The tree as installed for this Nav2 release, loaded with Nav2's plugins and ours.
  target_compile_definitions(test_behavior_trees PRIVATE
    LANDFALL_NAV2_TREE="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/${_landfall_tree_dir}/navigate_to_pose_w_relocalization.xml"
    LANDFALL_NAV2_PLUGIN="$<TARGET_FILE:landfall_nav2_bt_nodes>")
endif()

ament_package()
