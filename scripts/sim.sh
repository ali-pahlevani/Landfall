#!/usr/bin/env bash
# Runs a command in the simulation container (docker/Dockerfile.sim: Gazebo Harmonic, Nav2 and the
# TurtleBot 4 simulation) with the GPU, this repository and the Jazzy workspace built by
# scripts/docker_ci.sh. Not used by CI.
#
#   scripts/sim.sh '<command>'     e.g. scripts/sim.sh 'ros2 topic list'
#
# Mounted: the repository (read-only) at /ws/src/landfall, .docker_ws/jazzy/install at
# /ws/install (sourced), .docker_ws/sim/data at /data (outputs) and .docker_ws/sim/fuel at /fuel
# (Gazebo Fuel model cache: the depot is downloaded on the first run).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
distro=jazzy
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

docker run --rm ${LANDFALL_SIM_DOCKER_ARGS:-} \
  "${gpu_args[@]}" \
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
