cmake_minimum_required(VERSION 3.22)
project(landfall_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hypothesis.msg"
  "msg/RecorderStatus.msg"
  "msg/RelocalizationResult.msg"
  "msg/RelocalizationScores.msg"
  "srv/Relocalize.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
