#!/usr/bin/env bash
# Download the learned models (landfall_tools/models.yaml) and check their SHA-256.
#
#   scripts/fetch_models.sh [--models-dir DIR] [--parity] [FILE ...]
#
# See landfall_tools/fetch_models.py and docs/models.md.
set -euo pipefail
exec python3 "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/landfall_tools/fetch_models.py" "$@"
