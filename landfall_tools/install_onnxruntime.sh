#!/usr/bin/env bash
# Installs an official ONNX Runtime release for builds that need another one than rosdep's
# (onnxruntime_vendor ships the CPU build of the same version): the GPU build for the experimental
# CUDA execution provider (docs/models.md; outside the release guarantees, PLAN D11). The archive's
# SHA-256 is checked before anything is unpacked.
#
#   landfall_tools/install_onnxruntime.sh <cpu|gpu|aarch64> [prefix, default ~/.local/opt]
#
# Then build Landfall against it (the libraries find it through their RPATH):
#   colcon build --cmake-args -DONNXRUNTIME_ROOT=<prefix>/onnxruntime-linux-<...>-1.24.3
#
# gpu: CUDA 12.x and cuDNN 9.x, which NVIDIA distributes (closed source) and you install yourself;
# ONNX Runtime 1.21 to 1.26 are built with CUDA 12.8.
set -euo pipefail

version=1.24.3
variant="${1:?usage: install_onnxruntime.sh <cpu|gpu|aarch64> [prefix]}"
prefix="${2:-${HOME}/.local/opt}"
# SHA-256 of the release assets, as GitHub reports them and as downloaded on 2026-11-07.
case "${variant}" in
  cpu)
    name="onnxruntime-linux-x64-${version}"
    sha256=4c436a280d650f8bf32c921a2bf4de7c42cc32884c51c90e47de991708bbb5a4
    ;;
  gpu)
    name="onnxruntime-linux-x64-gpu-${version}"
    sha256=87c1cb812688c8f267bf2e3eba6ce6e174df97a323ba2c3c1d59a0a0f9ee140b
    ;;
  aarch64)
    name="onnxruntime-linux-aarch64-${version}"
    sha256=15100fb88b4c692cdd6bf2cca5f4a26a3806cebca8136de6681e2aba4b2ea033
    ;;
  *)
    echo "unknown variant '${variant}': cpu, gpu or aarch64" >&2
    exit 2
    ;;
esac

if [[ -e "${prefix}/${name}" ]]; then
  echo "${prefix}/${name} exists already" >&2
  exit 1
fi
work="$(mktemp -d)"
trap 'rm -rf "${work}"' EXIT
curl -fL --retry 3 -o "${work}/${name}.tgz" \
  "https://github.com/microsoft/onnxruntime/releases/download/v${version}/${name}.tgz"
echo "${sha256}  ${work}/${name}.tgz" | sha256sum --check --strict -
mkdir -p "${prefix}"
tar -xzf "${work}/${name}.tgz" -C "${prefix}"
echo "ONNX Runtime ${version} (${variant}) in ${prefix}/${name}"
echo "build with: colcon build --cmake-args -DONNXRUNTIME_ROOT=${prefix}/${name}"
