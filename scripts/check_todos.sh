#!/usr/bin/env bash
# Enforce the "no silent stubs" rule: every TODO/FIXME in the repository must be written as
# TODO(T-NNN) and T-NNN must have an entry in docs/TODO.md.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

status=0
while IFS= read -r line; do
  # References to the register itself ("docs/TODO.md") are not TODO markers.
  if ! grep -q -E '\b(TODO|FIXME)\b' <<< "${line//TODO.md/}"; then
    continue
  fi
  if ! grep -q -E 'TODO\(T-[0-9]{3}\)' <<< "${line}"; then
    echo "untracked TODO (write it as TODO(T-NNN) and register it in docs/TODO.md): ${line}"
    status=1
    continue
  fi
  while IFS= read -r id; do
    if ! grep -q -E "^\| ${id} \|" docs/TODO.md; then
      echo "${id} is not registered in docs/TODO.md: ${line}"
      status=1
    fi
  done < <(grep -o -E 'T-[0-9]{3}' <<< "${line}")
done < <(grep -r -n -I -E '\b(TODO|FIXME)\b' \
  --exclude-dir=.git --exclude-dir=.docker_ws --exclude-dir=build --exclude-dir=install \
  --exclude-dir=log --exclude-dir=docs --exclude='check_todos.sh' . || true)

if [[ "${status}" -eq 0 ]]; then
  echo "TODO check passed"
fi
exit "${status}"
