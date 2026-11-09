#!/usr/bin/env bash
# The kidnap benchmark in the depot simulation (docs/benchmark.md), inside the simulation
# container (scripts/sim.sh). Starts the simulation in one of the depot's conditions with Nav2 and
# Landfall's localization (sim/launch/depot_navigation.launch.xml; Nav2 stays idle), a bridge for
# Gazebo's set_pose service, and landfall_benchmark's kidnap_runner, which appends one row per
# placement to the output file.
#
#   benchmark_depot.sh <output.csv> <condition> <method> <label> <database> [runner arguments...]
#
#   condition  nominal, lighting, added_objects or crates_removed (sim/worlds/depot_<condition>.sdf)
#   method     landfall, amcl or watchdog; the relocalizer's kidnap watchdog is on for watchdog only
#   label      the scenario's name in the results, e.g. "nominal (classical)"
#   database   the map database of the relocalizer (classical or learned). The placements are
#              drawn near the keyframes of SAMPLING_DATABASE (default: this database), so that every
#              method meets the same ones.
# Runner arguments (e.g. -p placements:=10) override landfall_benchmark's scenarios/depot.yaml.
# A learned database needs its models: LANDFALL_MODELS_DIR, see docs/models.md.
set -euo pipefail
# Job control: background jobs keep the default SIGINT handling (see record_depot.sh).
set -m

out="${1:?usage: benchmark_depot.sh <output.csv> <condition> <method> <label> <database>}"
condition="${2:?condition}"
method="${3:?method}"
label="${4:?label}"
database="${5:?database}"
shift 5
share="$(ros2 pkg prefix landfall_bringup)/share/landfall_bringup"
scenario="$(ros2 pkg prefix landfall_benchmark)/share/landfall_benchmark/scenarios/depot.yaml"
map="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/maps/depot.yaml"
world=""
case "${condition}" in
  nominal) ;;
  lighting | added_objects | crates_removed) world="${share}/sim/worlds/depot_${condition}.sdf" ;;
  *)
    echo "unknown condition: ${condition}" >&2
    exit 1
    ;;
esac
relocalizer_params="${share}/sim/params/relocalizer_depot_sim.yaml"
if [[ "${method}" == watchdog ]]; then
  relocalizer_params="${share}/sim/params/relocalizer_depot_watchdog.yaml"
fi
mkdir -p "$(dirname "${out}")"
log="${out%.csv}_${condition}_${method}.log"

pids=()
stop() {
  # Every background job is a process group of its own (set -m): interrupt the whole group, as
  # Ctrl-C does. `ros2 run` waits for its child, expecting the child to get the signal too. What
  # still runs after 20 s is killed: a node stuck in a service call, or Gazebo, can hang there.
  for pid in "${pids[@]}"; do kill -INT -- "-${pid}" 2> /dev/null || true; done
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

WORLD="${world}" SIM_LAUNCH="${share}/sim/launch/depot_navigation.launch.xml" \
  bash "${share}/sim/scripts/start_depot_sim.sh" database:="${database}" map:="${map}" \
  relocalizer_params_file:="${relocalizer_params}" > "${log}.sim" 2>&1 &
pids+=($!)
ros2 run ros_gz_bridge parameter_bridge \
  /world/depot/set_pose@ros_gz_interfaces/srv/SetEntityPose > "${log}.bridge" 2>&1 &
pids+=($!)

# The runner waits for the services, the ground truth and AMCL's first pose (from the
# relocalizer's startup relocalization).
ros2 run landfall_benchmark kidnap_runner --ros-args --params-file "${scenario}" \
  -p method:="${method}" -p label:="${label}" -p output:="${out}" \
  -p sampling.map_yaml:="${map}" -p sampling.database:="${SAMPLING_DATABASE:-${database}}" \
  "$@" 2>&1 | tee "${log}"
