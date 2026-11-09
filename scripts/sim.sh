#!/usr/bin/env bash
# Runs a command in the simulation container (docker/Dockerfile.sim: Gazebo, Nav2 and the
# TurtleBot 4 simulation) with the GPU, this repository and the workspace built by
# scripts/docker_ci.sh. Not used by CI.
#
#   scripts/sim.sh '<command>'     e.g. scripts/sim.sh 'ros2 topic list'
#
# LANDFALL_SIM_DISTRO picks the ROS distro (default jazzy, the reference; humble runs the
# TurtleBot 4's Gazebo Fortress simulation). Mounted: the repository (read-only) at
# /ws/src/landfall, .docker_ws/<distro>/install at /ws/install (sourced), .docker_ws/sim/data at
# /data (outputs) and .docker_ws/sim/fuel at /fuel (Gazebo Fuel model cache: the depot is
# downloaded on the first run). LANDFALL_SIM_DOCKER_ARGS adds docker arguments, e.g. the learned
# models: -v ~/.local/share/landfall/models:/models:ro -e LANDFALL_MODELS_DIR=/models.
# Humble runs on Cyclone DDS (LANDFALL_SIM_RMW picks another RMW).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
distro="${LANDFALL_SIM_DISTRO:-jazzy}"
image="landfall-sim:${distro}"
sim_dir="${repo_root}/.docker_ws/sim"

if ! docker image inspect "landfall-deps:${distro}" > /dev/null 2>&1; then
  docker build --target deps --build-arg ROS_DISTRO="${distro}" \
    -t "landfall-deps:${distro}" -f "${repo_root}/docker/Dockerfile" "${repo_root}"
fi
if [[ "${LANDFALL_REBUILD_IMAGE:-0}" == 1 ]] || ! docker image inspect "${image}" > /dev/null 2>&1; then
  docker build -f "${repo_root}/docker/Dockerfile.sim" --build-arg BASE="landfall-deps:${distro}" \
    -t "${image}" "${repo_root}/docker"
fi
if [[ ! -f "${repo_root}/.docker_ws/${distro}/install/setup.bash" ]]; then
  echo "build the workspace first: scripts/docker_ci.sh ${distro}" >&2
  exit 1
fi
mkdir -p "${sim_dir}/data" "${sim_dir}/fuel" "${sim_dir}/home"

gpu_args=()
if docker info --format '{{json .Runtimes}}' 2> /dev/null | grep -q nvidia; then
  gpu_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
fi
# Humble: Cyclone DDS (see docker/Dockerfile.sim); LANDFALL_SIM_RMW picks another.
rmw_args=()
if [[ "${distro}" == humble ]]; then
  rmw_args=(-e RMW_IMPLEMENTATION="${LANDFALL_SIM_RMW:-rmw_cyclonedds_cpp}")
fi

docker run --rm ${LANDFALL_SIM_DOCKER_ARGS:-} \
  "${gpu_args[@]}" \
  "${rmw_args[@]}" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -v "${repo_root}:/ws/src/landfall:ro" \
  -v "${repo_root}/.docker_ws/${distro}/install:/ws/install:ro" \
  -v "${sim_dir}/data:/data" \
  -v "${sim_dir}/fuel:/fuel" \
  -v "${sim_dir}/home:/home/sim" \
  -w /data \
  "${image}" \
  setpriv --reuid="$(id -u)" --regid="$(id -g)" --clear-groups -- \
  env HOME=/home/sim bash -c "source /opt/ros/${distro}/setup.bash && source /ws/install/setup.bash && $*"
