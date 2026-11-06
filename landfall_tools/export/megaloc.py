# Copyright 2026 Ali Pahlevani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""MegaLoc for landfall_core's `megaloc` backend: upstream's own ONNX export, checked.

MegaLoc: huggingface.co/gberton/MegaLoc (MIT; DINOv2 backbone, Apache-2.0), revision pinned
below. Its repository ships megaloc.onnx; Landfall uses that file as it is.

    python3 -I megaloc.py --onnx megaloc.onnx --output DIR [--parity-images IMAGE ...]
                          [--pytorch DIR]

Copies megaloc.onnx to DIR after checking its SHA-256. With --parity-images, megaloc.parity.bin
holds the images (BGR) and the descriptors ONNX Runtime computes on them, preprocessed as
landfall_core's OnnxGlobalDescriptor does (322 x 322 with cv::INTER_AREA, RGB, ImageNet mean and
std; see common.py). With --pytorch DIR (upstream's megaloc_model.py and model.safetensors at
the same revision; this runs upstream's model code), the ONNX descriptors are also compared with
PyTorch's.
"""

import argparse
import pathlib
import shutil
import sys

import cv2
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # python3 -I drops it
import common  # noqa: E402

REVISION = 'a0f34722c4297ff787e022433799250180860af7'
ONNX_SHA256 = '9304d7d2473b006d2664736b5c56135c8404ff351d4592cfd9dcb4d7e8f12355'
SAFETENSORS_SHA256 = 'd4f9f2bcb60018f91eb6a8e061ed054fd55654e10c2569cf13841ea986ffb4f8'
SIZE = 322
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def preprocess(bgr):
    """OnnxGlobalDescriptor's input for megaloc: 322 x 322, RGB, normalized, NCHW."""
    small = cv2.resize(bgr, (SIZE, SIZE), interpolation=cv2.INTER_AREA)
    rgb = small[:, :, ::-1].astype(np.float32) / 255.0
    return ((rgb - MEAN) / STD).transpose(2, 0, 1)[None].astype(np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--onnx', required=True, type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--parity-images', nargs='*', default=[], type=pathlib.Path)
    parser.add_argument('--pytorch', type=pathlib.Path)
    args = parser.parse_args()

    if common.sha256(args.onnx) != ONNX_SHA256:
        raise SystemExit(f'{args.onnx}: not megaloc.onnx of gberton/MegaLoc@{REVISION}')
    args.output.mkdir(parents=True, exist_ok=True)
    model_path = args.output / 'megaloc.onnx'
    if model_path.resolve() != args.onnx.resolve():
        shutil.copyfile(args.onnx, model_path)

    import onnxruntime
    session = onnxruntime.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    print(f'inputs {[(i.name, i.shape) for i in inputs]}, '
          f'outputs {[(o.name, o.shape) for o in outputs]}')
    if len(inputs) != 1 or len(outputs) != 1:
        raise SystemExit('megaloc.onnx must map one input to one output')

    torch_model = None
    if args.pytorch:
        common.verify_files(args.pytorch, {'model.safetensors': SAFETENSORS_SHA256})
        from safetensors.torch import load_file
        sys.path.insert(0, str(args.pytorch))
        from megaloc_model import MegaLoc  # noqa: E402 (upstream, same revision)
        torch_model = MegaLoc().eval()
        torch_model.load_state_dict(load_file(str(args.pytorch / 'model.safetensors')))

    writer = None
    if args.parity_images:
        writer = common.ParityWriter(args.output / 'megaloc.parity.bin', common.KIND_GLOBAL)
    for path in args.parity_images:
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        x = preprocess(bgr)
        descriptor = session.run(None, {inputs[0].name: x})[0][0]
        descriptor = descriptor / np.linalg.norm(descriptor)
        if torch_model is not None:
            import torch
            with torch.no_grad():
                reference = torch_model(torch.from_numpy(x))[0].numpy()
            cosine = float(np.dot(descriptor, reference))
            print(f'{path.name}: ONNX vs PyTorch cosine {cosine:.6f}')
        writer.add_global(bgr, descriptor)
        print(f'{path.name}: descriptor of {descriptor.shape[0]}')
    if writer is not None:
        writer.close()

    for path in sorted(args.output.glob('megaloc*')):
        print(f'{path.name} {path.stat().st_size} bytes sha256 {common.sha256(path)}')


if __name__ == '__main__':
    main()
