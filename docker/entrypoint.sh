#!/usr/bin/env bash
# Entry point of the Landfall runtime image: source ROS and the installed workspace, then run CMD.
set -e
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source /opt/landfall/setup.bash
exec "$@"
