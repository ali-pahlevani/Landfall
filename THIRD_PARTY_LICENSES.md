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
| Ceres Solver | `landfall_core_build` (offline map builder) | BSD-3-Clause | Ubuntu's `libceres` links SuiteSparse, parts of which are LGPL/GPL; only `landfall_core_build` links it, never the runtime `landfall_core` used by the relocalizer |
| Eigen | `landfall_core` | MPL-2.0 | headers used unmodified |
| OpenCV | `landfall_core` and tests | Apache-2.0 | linked by explicit module list: core, imgproc, imgcodecs, features2d, calib3d (no videoio/highgui). SIFT's patent expired in 2020 |
| PoseLib 2.0.5 | `landfall_core` (compiled in, static, symbols hidden) | BSD-3-Clause | downloaded at build time from the GitHub release tarball, SHA-256 pinned in `landfall_core/cmake/PoseLib.cmake` |
| SQLite | `landfall_core` (map database) | public domain | system `libsqlite3` |
| yaml-cpp | `landfall_core` (map YAML) | MIT | system `libyaml-cpp` |
| GoogleTest | unit tests | BSD-3-Clause | via `ament_cmake_gtest` |
| class_loader, rosgraph_msgs | tests (component loading, simulated clock) | BSD-3-Clause / Apache-2.0 | |
| rclcpp, rclcpp_lifecycle, rclcpp_components, rosbag2, tf2, tf2_ros | `landfall_ros`, `landfall_benchmark` | Apache-2.0 / BSD-3-Clause (tf2) | |
| cv_bridge | `landfall_ros` | Apache-2.0 / BSD | |
| bondcpp, diagnostic_updater | `landfall_ros` | BSD-3-Clause | |
| nav2_behavior_tree | `landfall_nav2` | Apache-2.0 | |
| BehaviorTree.CPP | `landfall_nav2` | MIT | v3 on Humble, v4 on Jazzy and later |
| ROS 2 message packages | `landfall_msgs` and nodes | Apache-2.0 | `builtin_interfaces`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `std_srvs`, `lifecycle_msgs`, `nav_msgs`, `nav2_msgs` (relocalizer: AMCL's `SetInitialPose`), `tf2_msgs`, `visualization_msgs`, `diagnostic_msgs` |

## Data and assets in the repository

| Asset | Where | License | Notes |
|---|---|---|---|
| Depot occupancy map (`depot.yaml`, `depot.pgm`) | `landfall_ros/test/fixtures/depot_small/` | Apache-2.0 | from Nav2's `nav2_bringup` 1.3.13 (Open Navigation LLC and contributors), unmodified |
| Depot camera images in the fixture bag | `landfall_ros/test/fixtures/depot_small/bag/` | CC-BY-4.0 | renders of the "Depot" model by Open Robotics, Gazebo Fuel (`https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot`) |
| TurtleBot 4 description (derived) | `landfall_bringup/sim/urdf/` | Apache-2.0 | derived from `nav2_minimal_tb4_description` (nav2_minimal_turtlebot_simulation, Open Navigation LLC and contributors): 640 × 480 camera and a ground-truth pose publisher; the changes are noted in the files |

## Used at test or simulation time only (separate processes, not linked)

| Component | License | Notes |
|---|---|---|
| nav2_map_server, nav2_lifecycle_manager | Apache-2.0 | map parity and relocalization integration tests (`nav2_map_server` reads images with GraphicsMagick, MIT-style license) |
| nav2_amcl | LGPL-2.1+ | relocalization integration test: AMCL converges after the relocalizer's pose (a separate process, never linked) |
| Gazebo Harmonic, ros_gz, Nav2 (incl. AMCL, LGPL-2.1+), nav2_minimal_tb4_sim | Apache-2.0 (AMCL: LGPL-2.1+) | simulation image only (`docker/Dockerfile.sim`) |
| Python: rosbag2_py, OpenCV, NumPy | Apache-2.0, BSD-3-Clause | `landfall_tools/fixtures/depot_fixture.py` |

## Build and development tools (not distributed)

| Component | License |
|---|---|
| ament_cmake, ament_lint | Apache-2.0 |
| clang-format, clang-tidy 18 | Apache-2.0 with LLVM exception |
| ROS base Docker images (`ros:<distro>-ros-base`) | Apache-2.0 (Dockerfiles); contain Ubuntu packages under their own licenses |
