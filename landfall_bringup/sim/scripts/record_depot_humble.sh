#!/usr/bin/env bash
# Records the depot drive on Humble, inside the Humble simulation container
# (LANDFALL_SIM_DISTRO=humble scripts/sim.sh); the counterpart of record_depot.sh:
#   - starts the TurtleBot 4 simulator at the route's start (start_depot_sim_humble.sh) with
#     Landfall's localization, without a database (the relocalizer answers NOT_READY), and Nav2,
#     and gives AMCL the spawn pose;
#   - runs the Landfall recorder on the raw camera (staging session in <output>/sessions/depot);
#   - records a small bag: TF, AMCL, the scan and the simulator's ground truth (the raw camera
#     would take gigabytes; the recorder keeps the keyframes);
#   - drives the M2 route with Nav2, forwards and then backwards. The route is in the Jazzy depot
#     map (sim/routes/depot_loop.txt); turtlebot4_navigation's map is that one shifted by -8 m in x.
#
#   record_depot_humble.sh <output directory> [route file]
set -euo pipefail
# Job control: background jobs keep the default SIGINT handling (see record_depot.sh).
set -m

out="${1:?usage: record_depot_humble.sh <output directory> [route file]}"
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
route="${2:-${share}/sim/routes/depot_loop.txt}"
mkdir -p "${out}"
if [[ -e "${out}/bag" || -e "${out}/sessions" ]]; then
  echo "${out} already holds a recording" >&2
  exit 1
fi

pids=()
stop() {
  # SIGINT lets the recorder close its session and rosbag2 finish its metadata; every background
  # job is a process group of its own.
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

# The route starts at the Jazzy map's origin: (-8, 0) here, facing +x.
X_POSE=-8.0 Y_POSE=0.0 YAW=0.0 bash "${share}/sim/scripts/start_depot_sim_humble.sh" \
  startup_relocalization:=false > "${out}/sim.log" 2>&1 &
pids+=($!)

echo "waiting for AMCL..."
until ros2 lifecycle get /amcl 2> /dev/null | grep -q '^active'; do sleep 2; done
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -8.0}, orientation: {w: 1.0}},
    covariance: [0.0025, 0, 0, 0, 0, 0, 0, 0.0025, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.003]}}" > /dev/null
echo "waiting for Nav2..."
until ros2 lifecycle get /bt_navigator 2> /dev/null | grep -q '^active'; do sleep 2; done

"$(ros2 pkg prefix landfall_ros)/lib/landfall_ros/recorder_node" --ros-args \
  --params-file "${share}/sim/params/recorder_depot_humble.yaml" -p autostart:=true \
  -p output_dir:="${out}/sessions" -p session_name:=depot > "${out}/recorder.log" 2>&1 &
pids+=($!)

ros2 bag record -s sqlite3 -o "${out}/bag" /tf /tf_static /amcl_pose /odom /scan \
  /sim_ground_truth_pose /oakd/rgb/preview/camera_info > "${out}/bag.log" 2>&1 &
pids+=($!)
sleep 5

# NavigateThroughPoses goal from a leg file, as in record_depot.sh.
goal() {
  awk 'BEGIN { n = 0 } NF >= 2 { x[n] = $1; y[n] = $2; n++ }
    END {
      printf "{poses: ["
      for (i = 1; i < n; i++) {
        j = (i + 1 < n) ? i + 1 : i; k = (i + 1 < n) ? i : i - 1
        yaw = atan2(y[j] - y[k], x[j] - x[k])
        printf "%s{header: {frame_id: map}, pose: {position: {x: %s, y: %s}, orientation: {z: %.6f, w: %.6f}}}",
          (i > 1 ? ", " : ""), x[i], y[i], sin(yaw / 2), cos(yaw / 2)
      }
      printf "]}"
    }' "$1"
}

# The legs, shifted into this map, then the same legs backwards.
legs_dir="$(mktemp -d)"
awk -v dir="${legs_dir}" 'BEGIN { leg = 0 } /^---/ { leg++; next } !/^#/ && NF >= 2 {
  printf "%s %s\n", $1 - 8.0, $2 > sprintf("%s/forward_%02d", dir, leg) }' "${route}"
forward=("${legs_dir}"/forward_*)
legs=("${forward[@]}")
for ((i = ${#forward[@]} - 1; i >= 0; i--)); do
  tac "${forward[i]}" > "${forward[i]/forward_/backward_}"
  legs+=("${forward[i]/forward_/backward_}")
done

for leg in "${legs[@]}"; do
  echo "driving $(basename "${leg}"): $(($(wc -l < "${leg}") - 1)) waypoints"
  result="$(timeout 1800 ros2 action send_goal /navigate_through_poses \
    nav2_msgs/action/NavigateThroughPoses "$(goal "${leg}")" 2>&1 || true)"
  echo "${result}" | tail -n 1
  if ! echo "${result}" | grep -q 'SUCCEEDED'; then
    echo "navigation failed; stopping the recording" >&2
    exit 1
  fi
done
sleep 2
echo "drive complete"
