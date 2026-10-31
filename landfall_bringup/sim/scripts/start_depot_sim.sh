#!/usr/bin/env bash
# Starts the depot simulation inside the simulation container (scripts/sim.sh): Gazebo Harmonic
# with GPU headless rendering, the TurtleBot 4 with Landfall's 640 x 480 camera
# (sim/urdf/landfall_tb4.urdf.xacro), Nav2 with AMCL on nav2_bringup's depot map, and a bridge of
# Gazebo's ground-truth robot pose to /ground_truth/pose. Runs until interrupted.
#
# nav2_bringup's tb4_simulation_launch.py starts `gz sim -s` without --headless-rendering, which
# renders no camera images without a display; so the launch runs with use_simulator:=False (it
# still starts Nav2, robot_state_publisher and the bridges) and this script starts the Gazebo
# server and spawns the robot itself.
set -euo pipefail

robot_name="${ROBOT_NAME:-nav2_turtlebot4}"
x_pose="${X_POSE:--8.0}"
y_pose="${Y_POSE:-0.0}"
yaw="${YAW:-0.0}"
share_tb4="$(ros2 pkg prefix nav2_minimal_tb4_sim)/share/nav2_minimal_tb4_sim"
share_bringup="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
world="${WORLD:-${share_tb4}/worlds/depot.sdf}"
work="$(mktemp -d)"
trap 'kill $(jobs -p) 2> /dev/null; wait; rm -rf "${work}"' EXIT

xacro -o "${work}/world.sdf" headless:=true "${world}"
# Worlds, and the share directory that resolves the robot's model://<package>/meshes URIs.
share_root="$(ros2 pkg prefix nav2_minimal_tb4_description)/share"
export GZ_SIM_RESOURCE_PATH="${share_tb4}/worlds:${share_root}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
gz sim -r -s --headless-rendering "${work}/world.sdf" &

ros2 launch nav2_bringup tb4_simulation_launch.py use_simulator:=False headless:=True \
  use_rviz:=False robot_name:="${robot_name}" \
  robot_sdf:="${share_bringup}/sim/urdf/landfall_tb4.urdf.xacro" "$@" &

ros2 run ros_gz_bridge parameter_bridge \
  "/model/${robot_name}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose" \
  --ros-args -r "/model/${robot_name}/pose:=/ground_truth/pose" -p use_sim_time:=true &

# Spawn once robot_state_publisher publishes the description.
until ros2 topic list 2> /dev/null | grep -qx /robot_description; do sleep 1; done
ros2 run ros_gz_sim create -name "${robot_name}" -topic robot_description \
  -x "${x_pose}" -y "${y_pose}" -z 0.01 -Y "${yaw}"
wait
