#!/usr/bin/env bash
# The M6 navigation run in the depot simulation (docs/nav2_integration.md), inside the simulation
# container (scripts/sim.sh). Positions are in the depot map's frame, which is Gazebo's world
# shifted by (+8, 0).
#   1. The robot spawns at (12.0, -0.75) facing +x, and AMCL has no pose: the relocalizer's
#      startup relocalization gives it one (sim/launch/depot_navigation.launch.xml).
#   2. NavigateToPose to (3.0, -5.5), facing -x.
#   3. Kidnap: Gazebo moves the robot to (21.6, -3.5), facing the east wall, without AMCL knowing.
#      NavigateToPose to (12.0, -0.75): the robot drives on from the wrong pose until the controller
#      fails to make progress, and the behavior tree's first recovery relocalizes, looking around,
#      clears the costmaps and plans again from the true pose.
# Nav2 notices a wrong pose only when the robot can't move: until then it drives on blind, and it
# may stop where the database holds no views (docs/nav2_integration.md).
# Writes the simulation log (Nav2's, the relocalizer's), each goal's result and a bag (AMCL, ground
# truth, the relocalizer's status, initial poses, TF) to the output directory.
#
#   navigate_depot.sh <output directory> <database>
set -euo pipefail
# Job control: background jobs keep the default SIGINT handling (see record_depot.sh).
set -m

out="${1:?usage: navigate_depot.sh <output directory> <database>}"
database="${2:?usage: navigate_depot.sh <output directory> <database>}"
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
map="${MAP:-$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/depot.yaml}"
robot_name="nav2_turtlebot4"
mkdir -p "${out}"
if [[ -e "${out}/bag" ]]; then
  echo "${out} already holds a run" >&2
  exit 1
fi

pids=()
sim_pid=""
stop() {
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2> /dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2> /dev/null || true; done
  if [[ -n "${sim_pid}" ]]; then
    kill -INT "${sim_pid}" 2> /dev/null || true
    wait "${sim_pid}" 2> /dev/null || true
  fi
}
trap stop EXIT

X_POSE=4.0 Y_POSE=-0.75 YAW=0.0 SIM_LAUNCH="${share}/sim/launch/depot_navigation.launch.xml" \
  bash "${share}/sim/scripts/start_depot_sim.sh" database:="${database}" map:="${map}" \
  > "${out}/sim.log" 2>&1 &
sim_pid=$!

ros2 bag record --use-sim-time -s sqlite3 -o "${out}/bag" \
  /tf /tf_static /odom /amcl_pose /ground_truth/pose /initialpose /landfall_relocalizer/status \
  > "${out}/bag.log" 2>&1 &
pids+=($!)

elapsed() { echo "$(( $(date +%s) - start ))s"; }
start=$(date +%s)
echo "waiting for the startup relocalization..."
until timeout 5 ros2 topic echo --once /landfall_relocalizer/status 2> /dev/null \
    > "${out}/startup_status.txt" && grep -q '^published: true' "${out}/startup_status.txt"; do
  if (( $(date +%s) - start > 300 )); then
    echo "no startup relocalization within 300 s" >&2
    exit 1
  fi
  sleep 2
done
echo "startup relocalization published after $(elapsed):"
grep -E '^(status|message):' "${out}/startup_status.txt"

# Nav2's costmaps activate once AMCL publishes map->odom, i.e. after that pose.
echo "waiting for Nav2..."
until ros2 lifecycle get /bt_navigator 2> /dev/null | grep -q '^active'; do sleep 2; done
echo "Nav2 active after $(elapsed)"
sleep 5

# navigate <name> <x> <y> <yaw in degrees>: NavigateToPose in the map frame, waits for the result.
navigate() {
  local q
  q="$(awk -v y="$4" 'BEGIN { a = y * atan2(0, -1) / 360; printf "z: %.6f, w: %.6f", sin(a), cos(a) }')"
  echo "goal $1: ($2, $3, $4 deg) at $(elapsed)"
  timeout 600 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $2, y: $3}, orientation: {${q}}}}}" \
    > "${out}/$1.txt" 2>&1 || true
  echo "goal $1 done at $(elapsed): $(grep -E 'Goal (rejected|finished)' "${out}/$1.txt" || echo 'no result')"
}

navigate goal1 3.0 -5.5 180
sleep 3
echo "kidnap at $(elapsed): Gazebo moves the robot to (21.6, -3.5), facing the east wall"
gz service -s /world/depot/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --timeout 5000 --req "name: \"${robot_name}\", position: {x: 13.6, y: -3.5, z: 0.01}, orientation: {w: 1.0}"
sleep 3
navigate goal2 12.0 -0.75 0

sleep 2
stop
trap - EXIT
echo "run complete: ${out}"
