#!/usr/bin/env bash
# Repository policies from docs/PLAN.md that no compiler or linter checks:
#   1. ament_target_dependencies is never used (deprecated; link exported targets instead).
#   2. Python lives only in landfall_tools/ and in package test/ directories (launch tests).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

status=0
excludes=(--exclude-dir=.git --exclude-dir=.docker_ws --exclude-dir=build --exclude-dir=install
  --exclude-dir=log --exclude-dir=docs)

if grep -r -n -I "${excludes[@]}" --include='CMakeLists.txt' --include='*.cmake' \
     -E 'ament_target_dependencies[[:space:]]*\(' .; then
  echo "policy: ament_target_dependencies is forbidden; use landfall_link_package() or exported targets"
  status=1
fi

while IFS= read -r file; do
  case "${file}" in
    ./landfall_tools/* | ./*/test/*) ;;
    *)
      echo "policy: Python outside landfall_tools/ and test/ directories: ${file}"
      status=1
      ;;
  esac
done < <(find . \( -path ./.git -o -path ./.docker_ws -o -path ./build -o -path ./install \
  -o -path ./log -o -path ./.venv \) -prune -o -type f -name '*.py' -print)

if [[ "${status}" -eq 0 ]]; then
  echo "policy check passed"
fi
exit "${status}"
