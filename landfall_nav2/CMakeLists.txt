cmake_minimum_required(VERSION 3.22)
project(landfall_nav2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(landfall_core REQUIRED)
find_package(landfall_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# Pick the BehaviorTree.CPP major version from the Nav2 release, not from whichever BT package
# happens to be installed (Humble can have both): Nav2 1.1.x (Humble) uses v3, 1.2+ uses v4.
if(nav2_behavior_tree_VERSION VERSION_LESS "1.2.0")
  set(LANDFALL_BTCPP_V4 0)
  set(_landfall_bt_package behaviortree_cpp_v3)
else()
  set(LANDFALL_BTCPP_V4 1)
  set(_landfall_bt_package behaviortree_cpp)
endif()
find_package(${_landfall_bt_package} REQUIRED)
message(STATUS "landfall_nav2: Nav2 ${nav2_behavior_tree_VERSION}, using ${_landfall_bt_package}")

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_compat_bt test/test_compat_bt.cpp)
  target_include_directories(test_compat_bt PRIVATE include)
  target_compile_definitions(test_compat_bt PRIVATE LANDFALL_BTCPP_V4=${LANDFALL_BTCPP_V4})
  landfall_link_package(test_compat_bt PLAIN
    nav2_behavior_tree ${_landfall_bt_package} landfall_msgs)
  landfall_set_cxx_standard(test_compat_bt)
  landfall_set_warnings(test_compat_bt)
endif()

ament_package()
