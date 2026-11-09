#!/usr/bin/env bash
# Starts the depot simulation on Humble inside the Humble simulation container
# (LANDFALL_SIM_DISTRO=humble scripts/sim.sh): the TurtleBot 4's own simulator on Gazebo Fortress
# (turtlebot4_ignition_bringup) with GPU headless rendering, the robot and its dock without the
# TurtleBot 4's localization and Nav2, a clock bridge, and Landfall with Nav2
# (sim/launch/depot_humble.launch.xml). Arguments of this script go to that launch. Runs until
# interrupted.
#
# turtlebot4_ignition.launch.py starts Gazebo with its GUI, which renders no camera images without
# a display, so this script starts the server itself, as turtlebot4_ignition_bringup's
# ignition.launch.py does. turtlebot4_navigation's depot map is in Gazebo's world frame (to about
# 5 cm); X_POSE, Y_POSE (m) and YAW (rad) place the robot there.
set -euo pipefail
# Ignition Transport's default partition is <host>:<user>. The container's user has no name
# (scripts/sim.sh runs it as the host's UID), and without one the server and its clients (the
# robot's spawner, the bridges) never discover each other. An explicit partition fixes that;
# navigate_depot_humble.sh's `ign service` call uses the same one.
export IGN_PARTITION="${IGN_PARTITION:-landfall_sim}"

x_pose="${X_POSE:-0.0}"
y_pose="${Y_POSE:-0.0}"
yaw="${YAW:-0.0}"
share() { echo "$(ros2 pkg prefix "$1")/share/$1"; }
bringup="$(share turtlebot4_ignition_bringup)"
# The resource path of ignition.launch.py: worlds, and the share directory holding the robot's and
# the dock's meshes.
IGN_GAZEBO_RESOURCE_PATH="${bringup}/worlds:$(share irobot_create_ignition_bringup)/worlds"
IGN_GAZEBO_RESOURCE_PATH+=":$(ros2 pkg prefix turtlebot4_description)/share"
IGN_GAZEBO_RESOURCE_PATH+=":$(ros2 pkg prefix irobot_create_description)/share"
export IGN_GAZEBO_RESOURCE_PATH
# And its system plugin path (ros_gz_sim's gz_sim.launch.py): the library path, where
# ign_ros2_control, which drives the Create 3's wheels, is.
IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}:}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH+="${LD_LIBRARY_PATH}"
nav2_params="$(mktemp --suffix .yaml)"
trap 'kill $(jobs -p) 2> /dev/null; wait; rm -f "${nav2_params}"' EXIT

# The Create 3's description asks its sensors for OGRE 1, which needs a display; OGRE 2 renders
# headless.
ign gazebo -s -r --headless-rendering --render-engine-server ogre2 "${bringup}/worlds/depot.sdf" &
ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock' &
ros2 launch turtlebot4_ignition_bringup turtlebot4_spawn.launch.py world:=depot model:=standard \
  x:="${x_pose}" y:="${y_pose}" yaw:="${yaw}" nav2:=false slam:=false localization:=false \
  rviz:=false &
# turtlebot4_spawn.launch.py starts the robot on its dock, 16 cm in front of it: in the way of any
# route (Nav2 reports a collision ahead), and an object the map doesn't have. The Jazzy depot has
# no dock: remove it once it is there.
(
  for _ in $(seq 180); do
    if ign model --list 2> /dev/null | grep -q standard_dock; then
      ign service -s /world/depot/remove --reqtype ignition.msgs.Entity \
        --reptype ignition.msgs.Boolean --timeout 5000 --req 'name: "standard_dock", type: MODEL'
      break
    fi
    sleep 1
  done
) &

# Humble's Nav2 parameter files list every behavior tree plugin, and the file wins over launch
# overlays: the TurtleBot 4's nav2.yaml with Landfall's plugin library added to the list. Its tree
# comes from navigation.launch.xml (the TurtleBot 4's file leaves it at Nav2's default).
sed -E 's/^( *)plugin_lib_names:$/&\n\1- landfall_nav2_bt_nodes/' \
  "$(share turtlebot4_navigation)/config/nav2.yaml" > "${nav2_params}"
ros2 launch "$(share landfall_bringup)/sim/launch/depot_humble.launch.xml" \
  nav2_params_file:="${nav2_params}" "$@" &
wait
