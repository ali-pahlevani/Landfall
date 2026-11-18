#!/usr/bin/env bash
# Records the depot drive inside the simulation container (scripts/sim.sh), as described in
# docs/recording.md:
#   - starts the simulation (start_depot_sim.sh), tunes AMCL for it (params/amcl_depot_sim.yaml)
#     and gives AMCL the spawn pose;
#   - records the camera as JPEG (/rgbd_camera/image/compressed, which the Gazebo image bridge
#     publishes through image_transport);
#   - runs the Landfall recorder (staging session in <output>/sessions/depot);
#   - records a rosbag2 (sqlite3) with every input of `landfall build --bag` and the ground truth;
#   - drives the route with Nav2 (NavigateThroughPoses), forwards and then backwards.
#
#   record_depot.sh <output directory> [route file]
#
# Another world of the TurtleBot 4 simulation (the warehouse), through the environment: WORLD (its
# SDF), MAP (its map yaml), X_POSE / Y_POSE / YAW (the spawn pose in Gazebo's frame, as
# start_depot_sim.sh), INITIAL_POSE ("x y yaw_rad", AMCL's first pose in the map frame; default
# "0 0 0", the depot's spawn), SESSION_NAME (default depot) and SPIN_AT_LEG_ENDS=1 (a full turn on
# the spot after every leg: the driving protocol's turns at aisle ends, docs/recording.md).
set -euo pipefail
# Job control: background jobs keep the default SIGINT handling (a non-interactive shell would
# start them with SIGINT ignored), so `kill -INT` stops them cleanly.
set -m

out="${1:?usage: record_depot.sh <output directory> [route file]}"
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
route="${2:-${share}/sim/routes/depot_loop.txt}"
mkdir -p "${out}"
if [[ -e "${out}/bag" || -e "${out}/sessions" ]]; then
  echo "${out} already holds a recording" >&2
  exit 1
fi

pids=()
stop() {
  # SIGINT lets the recorder close its session and rosbag2 finish its metadata.
  for pid in "${pids[@]}"; do kill -INT "${pid}" 2> /dev/null || true; done
  for pid in "${pids[@]}"; do wait "${pid}" 2> /dev/null || true; done
}
trap stop EXIT

map_args=()
if [[ -n "${MAP:-}" ]]; then
  map_args=(map:="${MAP}")
fi
bash "${share}/sim/scripts/start_depot_sim.sh" "${map_args[@]}" > "${out}/sim.log" 2>&1 &
sim_pid=$!

# Nav2's planner activates only once AMCL publishes map->odom, i.e. after the initial pose.
echo "waiting for AMCL..."
until ros2 lifecycle get /amcl 2> /dev/null | grep -q '^active'; do sleep 2; done
ros2 param load /amcl "${share}/sim/params/amcl_depot_sim.yaml" > /dev/null

# The robot spawns at the depot map's origin, facing +x (INITIAL_POSE for another world).
read -r ix iy iyaw <<< "${INITIAL_POSE:-0 0 0}"
qz="$(awk -v a="${iyaw}" 'BEGIN { printf "%.6f", sin(a / 2) }')"
qw="$(awk -v a="${iyaw}" 'BEGIN { printf "%.6f", cos(a / 2) }')"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: ${ix}, y: ${iy}},
    orientation: {z: ${qz}, w: ${qw}}},
    covariance: [0.0025, 0, 0, 0, 0, 0, 0, 0.0025, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.003]}}" > /dev/null
echo "waiting for Nav2..."
until ros2 lifecycle get /bt_navigator 2> /dev/null | grep -q '^active'; do sleep 2; done

"$(ros2 pkg prefix landfall_ros)/lib/landfall_ros/recorder_node" --ros-args \
  --params-file "${share}/sim/params/recorder_depot_sim.yaml" -p autostart:=true \
  -p output_dir:="${out}/sessions" -p session_name:="${SESSION_NAME:-depot}" \
  > "${out}/recorder.log" 2>&1 &
pids+=($!)

ros2 bag record --use-sim-time -s sqlite3 -o "${out}/bag" \
  /tf /tf_static /amcl_pose /odom /imu /scan /ground_truth/pose \
  /rgbd_camera/image/compressed /rgbd_camera/camera_info > "${out}/bag.log" 2>&1 &
pids+=($!)
sleep 5

# NavigateThroughPoses goal from a leg file (its first point is where the robot stands, so it is
# skipped); each waypoint faces the next one.
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

# Split the route into legs, then add the same legs backwards (last leg first, points reversed).
legs_dir="$(mktemp -d)"
awk -v dir="${legs_dir}" 'BEGIN { leg = 0 } /^---/ { leg++; next } !/^#/ && NF >= 2 {
  printf "%s %s\n", $1, $2 > sprintf("%s/forward_%02d", dir, leg) }' "${route}"
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
  if [[ "${SPIN_AT_LEG_ENDS:-0}" == 1 ]]; then
    echo "turning on the spot"
    timeout 120 ros2 action send_goal /spin nav2_msgs/action/Spin "{target_yaw: 6.283}" \
      2>&1 | tail -n 1 || true
  fi
done
sleep 2
echo "drive complete"
stop
trap - EXIT
kill -INT "${sim_pid}" 2> /dev/null || true
wait "${sim_pid}" 2> /dev/null || true
