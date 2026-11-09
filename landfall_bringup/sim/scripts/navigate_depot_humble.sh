#!/usr/bin/env bash
# The M6 navigation run on Humble (docs/nav2_integration.md), inside the Humble simulation
# container (LANDFALL_SIM_DISTRO=humble scripts/sim.sh); the counterpart of navigate_depot.sh in
# turtlebot4_navigation's depot map (the Jazzy depot map shifted by -8 m in x, Gazebo's world):
#   1. The robot spawns at (4.0, -0.75) facing +x, and AMCL has no pose: the relocalizer's
#      startup relocalization gives it one.
#   2. NavigateToPose to (-5.0, -5.5), facing -x.
#   3. Kidnap: Gazebo moves the robot to (13.6, -3.5), facing the east wall, without AMCL knowing.
#      NavigateToPose to (4.0, -0.75): the behavior tree's first recovery relocalizes.
# Writes the simulation log, each goal's result and a bag (AMCL, ground truth, the relocalizer's
# status, initial poses, TF) to the output directory.
#
#   navigate_depot_humble.sh <output directory> <database>
set -euo pipefail
# Job control: background jobs keep the default SIGINT handling (see record_depot.sh).
set -m

out="${1:?usage: navigate_depot_humble.sh <output directory> <database>}"
database="${2:?usage: navigate_depot_humble.sh <output directory> <database>}"
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
mkdir -p "${out}"
if [[ -e "${out}/bag" ]]; then
  echo "${out} already holds a run" >&2
  exit 1
fi

pids=()
stop() {
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2> /dev/null || true; done
  # What still runs after 20 s is killed: after the first Humble recording, the simulation hadn't
  # exited 35 minutes after the SIGINT (benchmark_depot.sh does the same).
  for _ in $(seq 40); do
    local alive=0
    for pid in "${pids[@]}"; do
      if kill -0 -- "-${pid}" 2> /dev/null; then alive=1; fi
    done
    if ((alive == 0)); then break; fi
    sleep 0.5
  done
  for pid in "${pids[@]}"; do kill -KILL -- "-${pid}" 2> /dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2> /dev/null || true; done
}
trap stop EXIT

X_POSE=4.0 Y_POSE=-0.75 YAW=0.0 bash "${share}/sim/scripts/start_depot_sim_humble.sh" \
  database:="${database}" > "${out}/sim.log" 2>&1 &
pids+=($!)

ros2 bag record -s sqlite3 -o "${out}/bag" /tf /tf_static /odom /amcl_pose \
  /sim_ground_truth_pose /initialpose /landfall_relocalizer/status \
  > "${out}/bag.log" 2>&1 &
pids+=($!)

elapsed() { echo "$(($(date +%s) - start))s"; }
start=$(date +%s)
echo "waiting for the startup relocalization..."
until timeout 5 ros2 topic echo --once /landfall_relocalizer/status 2> /dev/null \
  > "${out}/startup_status.txt" && grep -q '^published: true' "${out}/startup_status.txt"; do
  if (($(date +%s) - start > 300)); then
    echo "no startup relocalization within 300 s" >&2
    exit 1
  fi
  sleep 2
done
echo "startup relocalization published after $(elapsed):"
grep -E '^(status|message):' "${out}/startup_status.txt"

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

navigate goal1 -5.0 -5.5 180
sleep 3
echo "kidnap at $(elapsed): Gazebo moves the robot to (13.6, -3.5), facing the east wall"
# The partition start_depot_sim_humble.sh gives the simulation.
IGN_PARTITION="${IGN_PARTITION:-landfall_sim}" ign service -s /world/depot/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean --timeout 5000 \
  --req 'name: "turtlebot4", position: {x: 13.6, y: -3.5, z: 0.01}, orientation: {w: 1.0}'
sleep 3
navigate goal2 4.0 -0.75 0

sleep 2
stop
trap - EXIT
echo "run complete: ${out}"
