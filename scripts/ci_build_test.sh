#!/usr/bin/env bash
# Build and test the workspace. Shared by GitHub Actions and scripts/docker_ci.sh so both run
# exactly the same steps. Expects to run where /opt/ros/$ROS_DISTRO exists and the sources are
# under $LANDFALL_WS/src (default workspace: /ws).
#
# Environment:
#   LANDFALL_WS          workspace root (default /ws)
#   LANDFALL_CMAKE_ARGS  extra CMake arguments, e.g. "-DCMAKE_CXX_STANDARD=20"
#   LANDFALL_SANITIZE    "1" builds with AddressSanitizer and UndefinedBehaviorSanitizer
#   LANDFALL_PACKAGES    only build and test these packages (and what they depend on)
set -eo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set (the official ROS images set it)}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

cd "${LANDFALL_WS:-/ws}"

cmake_args=(
  # Packages without C++ ignore the compiler options below. The leading space keeps colcon from
  # parsing this double-dash argument as one of its own options.
  " --no-warn-unused-cli"
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
  -DLANDFALL_WERROR=ON
)
if command -v ccache > /dev/null; then
  cmake_args+=(-DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
# shellcheck disable=SC2206  # word splitting of extra args is intended
cmake_args+=(${LANDFALL_CMAKE_ARGS:-})
if [[ "${LANDFALL_SANITIZE:-0}" == 1 ]]; then
  sanitize="-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer"
  # GCC's -Wmaybe-uninitialized gives false positives under sanitizer instrumentation (e.g. in
  # libstdc++'s std::regex); the regular builds keep checking it with -Werror.
  cmake_args+=(
    "-DCMAKE_CXX_FLAGS=${sanitize} -Wno-maybe-uninitialized"
    "-DCMAKE_EXE_LINKER_FLAGS=${sanitize}"
    "-DCMAKE_SHARED_LINKER_FLAGS=${sanitize}"
  )
fi
package_args=()
if [[ -n "${LANDFALL_PACKAGES:-}" ]]; then
  # shellcheck disable=SC2206  # a space-separated package list
  package_args=(--packages-up-to ${LANDFALL_PACKAGES})
fi

echo "=== Landfall CI: ROS ${ROS_DISTRO}, $(uname -m), CMake args: ${cmake_args[*]}"

colcon build --base-paths src --event-handlers console_cohesion+ "${package_args[@]}" \
  --cmake-args "${cmake_args[@]}"
colcon test --base-paths src --event-handlers console_cohesion+ "${package_args[@]}" \
  --return-code-on-test-failure
colcon test-result --verbose
