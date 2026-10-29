# Third-party licenses

Landfall is Apache-2.0. This file lists every third-party component it uses, with its license and
how it is used. It is updated in each milestone as components are added (full target list:
`docs/PLAN.md` §8 and §10).

Policy: only permissively licensed components are linked. GPL/LGPL software appears only as a
separate process (for example AMCL or slam_toolbox) or as a transitive system library of the
offline map builder. The only closed-source components are NVIDIA's, and only in the optional GPU
path.

## Linked or compiled in

| Component | Used by | License | Notes |
|---|---|---|---|
| Ceres Solver | `landfall_core` (Ceres compatibility layer; offline builder from M1) | BSD-3-Clause | Ubuntu's `libceres` links SuiteSparse, parts of which are LGPL/GPL; it is used only by the offline builder, never by the online relocalizer |
| Eigen | via Ceres (directly from M1) | MPL-2.0 | headers used unmodified |
| OpenCV | `landfall_ros` tests (all of Landfall from M1) | Apache-2.0 | linked by explicit module list (no videoio/highgui) |
| GoogleTest | unit tests | BSD-3-Clause | via `ament_cmake_gtest` |
| rclcpp, rclcpp_lifecycle, rosbag2 | `landfall_ros` | Apache-2.0 | |
| cv_bridge | `landfall_ros` | Apache-2.0 / BSD | |
| bondcpp, diagnostic_updater | `landfall_ros` | BSD-3-Clause | |
| nav2_behavior_tree | `landfall_nav2` | Apache-2.0 | |
| BehaviorTree.CPP | `landfall_nav2` | MIT | v3 on Humble, v4 on Jazzy and later |
| ROS 2 message packages | `landfall_msgs` and nodes | Apache-2.0 | `builtin_interfaces`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `lifecycle_msgs` |

## Build and development tools (not distributed)

| Component | License |
|---|---|
| ament_cmake, ament_lint | Apache-2.0 |
| clang-format, clang-tidy 18 | Apache-2.0 with LLVM exception |
| ROS base Docker images (`ros:<distro>-ros-base`) | Apache-2.0 (Dockerfiles); contain Ubuntu packages under their own licenses |
