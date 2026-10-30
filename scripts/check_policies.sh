#!/usr/bin/env bash
# Repository policies from docs/PLAN.md that no compiler or linter checks:
#   1. ament_target_dependencies is never used (deprecated; link exported targets instead).
#   2. Python lives only in landfall_tools/ and in package test/ directories (launch tests).
#   3. Nothing inside the source tree is git-ignored, so a commit never silently leaves out a
#      source file (only the workspace outputs listed below may be ignored).
#
# Files are taken from git's view (tracked plus untracked-but-not-ignored, i.e. what a commit of
# everything would contain), so build outputs are skipped wherever they are.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

status=0

if git grep --untracked -n -I -E 'ament_target_dependencies[[:space:]]*\(' \
     -- '*CMakeLists.txt' '*.cmake' ':!docs'; then
  echo "policy: ament_target_dependencies is forbidden; use landfall_link_package() or exported targets"
  status=1
fi

while IFS= read -r file; do
  case "${file}" in
    landfall_tools/* | */test/*) ;;
    *)
      echo "policy: Python outside landfall_tools/ and test/ directories: ${file}"
      status=1
      ;;
  esac
done < <(git ls-files --cached --others --exclude-standard -- '*.py')

# Ignored paths that may exist in a working copy: workspace outputs at the repository root and
# Python caches next to tests.
while IFS= read -r path; do
  case "${path}" in
    .docker_ws/ | build/ | Build/ | build-*/ | install/ | log/ | models/ | .venv/) ;;
    __pycache__/ | */__pycache__/ | *.pyc) ;;
    *)
      echo "policy: ${path} is inside the source tree but ignored by .gitignore (it would not be committed)"
      status=1
      ;;
  esac
done < <(git ls-files --others --ignored --exclude-standard --directory)

if [[ "${status}" -eq 0 ]]; then
  echo "policy check passed"
fi
exit "${status}"
