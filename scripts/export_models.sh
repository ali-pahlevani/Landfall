#!/usr/bin/env bash
# Export the learned models from their pinned upstream files, with the parity files of the C++
# tests (docs/models.md).
#
#   scripts/export_models.sh [OUTPUT_DIR]
#
# OUTPUT_DIR (default $LANDFALL_MODELS_DIR, else ~/.local/share/landfall/models, where Landfall
# looks for models) receives xfeat.onnx, eigenplaces_resnet18_512.onnx, megaloc.onnx, their
# *.parity.bin files and export.log: about 520 MB. Work files go to $LANDFALL_EXPORT_DIR (default
# ~/.cache/landfall/export): the virtual environment of landfall_tools/requirements.txt (about
# 1.2 GB), the upstream downloads and the parity images. They are kept for re-runs; delete the
# folder when done.
#
# This runs upstream code and loads upstream weights, every file checked against its SHA-256 in
# landfall_tools/models.yaml first: XFeat's model code and weights (verlab/accelerated_features)
# and the EigenPlaces checkpoint (gmberton/EigenPlaces), both through torch.load(weights_only=True)
# on torch 2.6.0, and MegaLoc's ONNX graph (gberton/MegaLoc) in ONNX Runtime.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
out=${1:-${LANDFALL_MODELS_DIR:-$HOME/.local/share/landfall/models}}
work=${LANDFALL_EXPORT_DIR:-$HOME/.cache/landfall/export}
mkdir -p "$out" "$work"
out=$(cd "$out" && pwd)
work=$(cd "$work" && pwd)
tools="$repo/landfall_tools"
exec > >(tee "$out/export.log") 2>&1

echo "== Python environment: $work/venv"
if [ ! -x "$work/venv/bin/python" ]; then
  python3 -m venv "$work/venv"
fi
# -I everywhere: no PYTHONPATH (a sourced ROS setup), no user site, no current directory.
py=("$work/venv/bin/python" -I)
"${py[@]}" -m pip install --quiet --disable-pip-version-check --no-cache-dir \
  -r "$tools/requirements.txt"
cd "$work"

echo "== Upstream files: $work/upstream"
"${py[@]}" "$tools/fetch_models.py" --upstream "$work/upstream" \
  xfeat.onnx eigenplaces_resnet18_512.onnx
"${py[@]}" "$tools/fetch_models.py" --models-dir "$out" megaloc.onnx

echo "== Parity images: $work/parity_images"
listing=$("${py[@]}" "$tools/export/parity_images.py" \
  --bag "$repo/landfall_ros/test/fixtures/depot_small/bag" --output "$work/parity_images")
mapfile -t images <<<"$listing"
printf '%s\n' "${images[@]}"

echo "== XFeat"
"${py[@]}" "$tools/export/xfeat.py" --upstream "$work/upstream/xfeat" --output "$out" \
  --parity-images "${images[@]}"

echo "== EigenPlaces"
"${py[@]}" "$tools/export/eigenplaces.py" \
  --weights "$work/upstream/eigenplaces/ResNet18_512_eigenplaces.pth" --output "$out" \
  --parity-images "${images[@]}"

echo "== MegaLoc"
"${py[@]}" "$tools/export/megaloc.py" --onnx "$out/megaloc.onnx" --output "$out" \
  --parity-images "${images[@]}"

echo "== Done: $out"
cd "$out"
sha256sum xfeat.onnx xfeat.parity.bin eigenplaces_resnet18_512.onnx eigenplaces.parity.bin \
  megaloc.onnx megaloc.parity.bin
