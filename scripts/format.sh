#!/usr/bin/env bash
# Format (or, with --check, verify) all C/C++ sources with the pinned clang-format 18.
#
#   scripts/format.sh           rewrite files in place
#   scripts/format.sh --check   fail if any file is not formatted
#
# Without a local clang-format 18 the script re-runs itself in the landfall-lint:jazzy image.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mode=fix
if [[ "${1:-}" == "--check" ]]; then
  mode=check
fi

find_clang_format() {
  local candidate
  for candidate in clang-format-18 clang-format; do
    if command -v "${candidate}" > /dev/null &&
       "${candidate}" --version | grep -q 'version 18\.'; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

if ! clang_format="$(find_clang_format)"; then
  if [[ -f /.dockerenv ]]; then
    echo "clang-format 18 not found in this container" >&2
    exit 1
  fi
  if [[ "${LANDFALL_REBUILD_IMAGE:-0}" == 1 ]] ||
     ! docker image inspect landfall-lint:jazzy > /dev/null 2>&1; then
    docker build -q --target lint --build-arg ROS_DISTRO=jazzy \
      -t landfall-lint:jazzy -f "${repo_root}/docker/Dockerfile" "${repo_root}" > /dev/null
  fi
  exec docker run --rm --user "$(id -u):$(id -g)" -v "${repo_root}:/src" -w /src \
    landfall-lint:jazzy scripts/format.sh "$@"
fi

cd "${repo_root}"
mapfile -t files < <(find . \
  \( -path ./.git -o -path ./.docker_ws -o -path ./build -o -path ./install -o -path ./log \) -prune \
  -o -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.cc' \) -print | sort)

echo "$("${clang_format}" --version | head -n1); ${#files[@]} files; mode=${mode}"
if [[ "${mode}" == check ]]; then
  "${clang_format}" --dry-run --Werror --style=file "${files[@]}"
  echo "format check passed"
else
  "${clang_format}" -i --style=file "${files[@]}"
fi
