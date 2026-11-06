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

# GitHub keeps job logs and artifacts behind a login, but annotations are public: on failure, the
# first compiler errors or the failed tests are also published as annotations (GitHub shows at
# most 10 per step).
annotate() {  # annotate TITLE MESSAGE
  [[ -n "${GITHUB_ACTIONS:-}" ]] || return 0
  local message="${2:0:4000}"
  message="${message//'%'/'%25'}"
  message="${message//$'\r'/'%0D'}"
  message="${message//$'\n'/'%0A'}"
  echo "::error title=${1//[:,]/ }::${message}"
}

annotate_build_failure() {
  local errors
  errors="$(grep -h -E 'error:|error [0-9]+|undefined reference|CMake Error' \
    log/latest_build/*/stderr.log 2> /dev/null | head -n 40 || true)"
  annotate "build failed (${ROS_DISTRO} $(uname -m))" "${errors:-no compiler error found; see the log}"
}

annotate_test_failures() {
  local line title="" block="" count=0
  while IFS= read -r line; do
    if [[ "${line}" =~ ^build/.*\ [0-9]+\ tests?, ]]; then
      if [[ -n "${title}" ]]; then
        annotate "${title}" "${block}"
        count=$((count + 1))
      fi
      title="${line}"
      block=""
      if ((count >= 9)); then
        title=""
        break
      fi
    elif [[ -n "${title}" ]]; then
      block+="${line}"$'\n'
    fi
  done < <(colcon test-result --verbose 2>&1 || true)
  if [[ -n "${title}" ]]; then
    annotate "${title}" "${block}"
  fi
}

if ! colcon build --base-paths src --event-handlers console_cohesion+ "${package_args[@]}" \
  --cmake-args "${cmake_args[@]}"; then
  annotate_build_failure
  exit 1
fi
if ! colcon test --base-paths src --event-handlers console_cohesion+ "${package_args[@]}" \
  --return-code-on-test-failure; then
  colcon test-result --verbose || true
  annotate_test_failures
  exit 1
fi
colcon test-result --verbose
