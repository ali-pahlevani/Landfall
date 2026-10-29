#!/usr/bin/env bash
# Entry point of the Docker helper scripts (docker_ci.sh, tidy.sh). Runs as root inside a
# landfall-deps or landfall-lint image: installs any rosdep dependencies the image lacks (added to
# a package.xml after the image was built) into this throwaway container, then drops to the
# calling user (HOST_UID/HOST_GID) and runs the given command.
#
# Topping up here instead of rebuilding the image keeps disk use flat: a rebuilt dependency layer
# costs ~1.7 GB per distro. Rebuild deliberately with LANDFALL_REBUILD_IMAGE=1 when the top-up
# grows large.
set -euo pipefail

: "${HOST_UID:?HOST_UID must be set}"
: "${HOST_GID:?HOST_GID must be set}"
src="${LANDFALL_SRC:-/ws/src}"

if ! rosdep check --from-paths "${src}" --ignore-src --rosdistro "${ROS_DISTRO}" > /dev/null 2>&1; then
  echo "=== image lacks some dependencies; installing them for this run only"
  apt-get update -qq
  rosdep install --from-paths "${src}" --ignore-src --rosdistro "${ROS_DISTRO}" -y -q
fi

exec setpriv --reuid="${HOST_UID}" --regid="${HOST_GID}" --clear-groups -- env HOME=/tmp "$@"
