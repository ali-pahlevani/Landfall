#!/usr/bin/env bash
# Try Landfall by hand in the depot simulation, with Gazebo's window and RViz, inside the simulation
# container started with the display (scripts/sim.sh):
#
#   LANDFALL_SIM_GUI=1 scripts/sim.sh \
#     'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/try_depot.sh /data/depot.landfall.db'
#
# It runs the depot, the TurtleBot 4 with Landfall's camera, Nav2 with the example behavior tree,
# and Landfall's localization (AMCL and the relocalizer at their defaults: the startup
# relocalization gives AMCL its first pose, the kidnap watchdog is on). Then:
#   - RViz's "Nav2 Goal" sends the robot somewhere; "2D Pose Estimate" gives AMCL a wrong pose, which
#     the kidnap watchdog notices and corrects within seconds;
#   - in Gazebo, select the robot and move it with the translate or rotate tool: a kidnap, which the
#     watchdog corrects too, once the camera sees a place the recording saw;
#   - in another terminal, `docker exec -it <container> bash` gives a shell as your user, like the
#     simulation (source /opt/ros/jazzy/setup.bash and /ws/install/setup.bash), for `ros2 service
#     call /landfall_relocalizer/relocalize landfall_msgs/srv/Relocalize "{}"` and
#     `ros2 topic echo /landfall_relocalizer/status`.
# The database is a build of the depot recording (docs/recording.md, "The simulation recording").
# Ctrl-C ends it.
#
#   try_depot.sh <database>
set -euo pipefail

database="${1:?usage: try_depot.sh <database>}"
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
map="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/depot.yaml"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The robot spawns at (12.0, -0.75) in the map frame (Gazebo's depot is shifted by 8 m in x).
export GUI=1
export X_POSE="${X_POSE:-4.0}" Y_POSE="${Y_POSE:--0.75}" YAW="${YAW:-0.0}"
export SIM_LAUNCH="${share}/sim/launch/depot_navigation.launch.xml"
exec bash "${here}/start_depot_sim.sh" database:="${database}" map:="${map}" \
  relocalizer_params_file:="${share}/sim/params/relocalizer_depot_watchdog.yaml"
