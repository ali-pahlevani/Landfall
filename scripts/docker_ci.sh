#!/usr/bin/env bash
# Build and test Landfall for one ROS distro inside Docker, the same way CI does.
#
#   scripts/docker_ci.sh <humble|jazzy|lyrical>
#   LANDFALL_VARIANT=cxx20 LANDFALL_CMAKE_ARGS="-DCMAKE_CXX_STANDARD=20" scripts/docker_ci.sh jazzy
#   LANDFALL_VARIANT=asan LANDFALL_SANITIZE=1 LANDFALL_PACKAGES=landfall_core scripts/docker_ci.sh jazzy
#   LANDFALL_REBUILD_IMAGE=1 scripts/docker_ci.sh jazzy   # refresh the dependency image
#
# The landfall-deps:<distro> image is built once and reused; dependencies added to a package.xml
# later are installed into each throwaway container by scripts/in_container.sh, so editing a
# manifest doesn't cost a new ~1.7 GB image layer.
#
# Build output stays in .docker_ws/<distro>[-<variant>]/ (git-ignored), so variants never share a
# CMake cache with the plain build. The source tree is mounted read-only, and the build itself
# runs as the calling user.
set -euo pipefail

distro="${1:?usage: $0 <humble|jazzy|lyrical>}"
case "${distro}" in
  humble | jazzy | lyrical) ;;
  *) echo "unsupported distro: ${distro}" >&2; exit 2 ;;
esac

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="landfall-deps:${distro}"
ws="${repo_root}/.docker_ws/${distro}${LANDFALL_VARIANT:+-${LANDFALL_VARIANT}}"

if [[ "${LANDFALL_REBUILD_IMAGE:-0}" == 1 ]] || ! docker image inspect "${image}" > /dev/null 2>&1; then
  docker build --target deps --build-arg ROS_DISTRO="${distro}" \
    -t "${image}" -f "${repo_root}/docker/Dockerfile" "${repo_root}"
fi

mkdir -p "${ws}/build" "${ws}/install" "${ws}/log" "${ws}/ccache"
docker run --rm \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e CCACHE_DIR=/ws/.ccache \
  -e LANDFALL_CMAKE_ARGS="${LANDFALL_CMAKE_ARGS:-}" \
  -e LANDFALL_SANITIZE="${LANDFALL_SANITIZE:-0}" \
  -e LANDFALL_PACKAGES="${LANDFALL_PACKAGES:-}" \
  -v "${repo_root}:/ws/src/landfall:ro" \
  -v "${ws}/build:/ws/build" \
  -v "${ws}/install:/ws/install" \
  -v "${ws}/log:/ws/log" \
  -v "${ws}/ccache:/ws/.ccache" \
  -w /ws \
  "${image}" \
  /ws/src/landfall/scripts/in_container.sh /ws/src/landfall/scripts/ci_build_test.sh
