#!/usr/bin/env bash
# Run clang-tidy 18 over Landfall's own C++ files, using the compile databases of a Jazzy build.
#
#   scripts/docker_ci.sh jazzy && scripts/tidy.sh
#
# Runs inside the landfall-lint:jazzy image with the same mounts as docker_ci.sh, so the paths in
# compile_commands.json resolve. Any finding fails (WarningsAsErrors in .clang-tidy).
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /.dockerenv ]]; then
  ws="${repo_root}/.docker_ws/jazzy"
  if [[ ! -d "${ws}/build" ]]; then
    echo "no Jazzy build found; run scripts/docker_ci.sh jazzy first" >&2
    exit 1
  fi
  if [[ "${LANDFALL_REBUILD_IMAGE:-0}" == 1 ]] ||
     ! docker image inspect landfall-lint:jazzy > /dev/null 2>&1; then
    docker build -q --target lint --build-arg ROS_DISTRO=jazzy \
      -t landfall-lint:jazzy -f "${repo_root}/docker/Dockerfile" "${repo_root}" > /dev/null
  fi
  # in_container.sh installs dependencies added since the image was built (their headers are
  # needed to analyze the sources), then runs this script again as the calling user.
  exec docker run --rm -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
    -v "${repo_root}:/ws/src/landfall:ro" \
    -v "${ws}/build:/ws/build:ro" \
    -v "${ws}/install:/ws/install:ro" \
    -w /ws landfall-lint:jazzy \
    /ws/src/landfall/scripts/in_container.sh /ws/src/landfall/scripts/tidy.sh "$@"
fi

build_dir="${LANDFALL_WS:-/ws}/build"
run_clang_tidy="$(command -v run-clang-tidy-18 || command -v run-clang-tidy)"
status=0
for db in "${build_dir}"/*/compile_commands.json; do
  package="$(basename "$(dirname "${db}")")"
  case "${package}" in
    landfall_msgs) continue ;;  # generated code only
  esac
  echo "=== clang-tidy: ${package}"
  # Only files of this repository; generated and system code is skipped.
  "${run_clang_tidy}" -quiet -p "$(dirname "${db}")" '/src/landfall/' || status=1
done
exit "${status}"
