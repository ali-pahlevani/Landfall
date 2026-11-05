#!/usr/bin/env bash
# Regenerate docs/parameters.md from the nodes' parameter declarations (Jazzy build in Docker).
#
#   scripts/update_parameters_doc.sh
#
# Builds landfall_ros like scripts/docker_ci.sh (reusing .docker_ws/jazzy), then runs
# `test_parameters_doc --print`. CI's test_parameters_doc fails while the committed file is stale.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="landfall-deps:jazzy"
ws="${repo_root}/.docker_ws/jazzy"
if ! docker image inspect "${image}" > /dev/null 2>&1; then
  docker build --target deps --build-arg ROS_DISTRO=jazzy \
    -t "${image}" -f "${repo_root}/docker/Dockerfile" "${repo_root}"
fi
mkdir -p "${ws}/build" "${ws}/install" "${ws}/log" "${ws}/ccache"
docker run --rm \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" -e CCACHE_DIR=/ws/.ccache \
  -v "${repo_root}:/ws/src/landfall:ro" \
  -v "${ws}/build:/ws/build" -v "${ws}/install:/ws/install" -v "${ws}/log:/ws/log" \
  -v "${ws}/ccache:/ws/.ccache" -w /ws "${image}" \
  /ws/src/landfall/scripts/in_container.sh bash -c '
    set -eo pipefail
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    colcon build --base-paths src --packages-up-to landfall_ros \
      --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      > /tmp/build.log 2>&1 || { tail -50 /tmp/build.log >&2; exit 1; }
    source install/setup.bash
    build/landfall_ros/test_parameters_doc --print > build/parameters.md 2> /dev/null'
cp "${ws}/build/parameters.md" "${repo_root}/docs/parameters.md"
echo "wrote docs/parameters.md"
