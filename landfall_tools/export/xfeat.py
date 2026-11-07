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

"""Export XFeat to ONNX for landfall_core's `xfeat` backend, with parity references.

XFeat: verlab/accelerated_features (Apache-2.0, code and weights), pinned below.

    python3 -I xfeat.py --upstream DIR --output DIR [--parity-images IMAGE ...] [--top-k 4096]

DIR is a copy of the upstream repository at UPSTREAM_COMMIT (at least the files in
UPSTREAM_FILES, each checked against its SHA-256). Writes xfeat.onnx and, with
--parity-images, xfeat.parity.bin: the images (grayscale) and what upstream's own
XFeat.detectAndCompute finds on them (see common.py).

The graph is XFeat's dense network plus the cheap dense steps of detectAndCompute: the L2
normalization of the descriptor map, the keypoint heatmap (softmax, 8x8 pixel shuffle) and the
5x5 local-maximum mask of non-maximum suppression. The threshold, the scores, the top-k and the
descriptor sampling run in C++ (landfall_core/src/onnx/xfeat.cpp), which follows
detectAndCompute step by step. One function of the network is swapped for the export:
XFeatModel._unfold2d becomes pixel_unshuffle (space_to_depth below), which torch.onnx can export
for any image size; the script checks that the network's outputs stay bit for bit the same.

    input  image        float32 [1, 1, H, W]    grayscale in [0, 1]; H and W multiples of 32
    output descriptors  float32 [1, 64, H/8, W/8]  L2-normalized over the channels
    output heatmap      float32 [1, 1, H, W]    keypoint probability
    output reliability  float32 [1, 1, H/8, W/8]
    output is_max       bool    [1, 1, H, W]    heatmap equals its 5x5 maximum
"""

import argparse
import pathlib
import sys

import cv2
import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # python3 -I drops it
import common  # noqa: E402

UPSTREAM_COMMIT = 'e92685f57f8318b18725c5c8c0bd28c7fe188d9a'
UPSTREAM_FILES = {
    'modules/model.py': 'd9a665f18fcea5eaf3e278925e1a92103afcba9051e05b2334f3daa29f411964',
    'modules/xfeat.py': '385ccd31d095b0d4176b04e982088b85321b11ade4324f83b097ee6524f2a6e7',
    'modules/interpolator.py': 'd63a6163eb6fff81e8720231f62537a42a69fccb44dc8851b04de5115daab4da',
    'weights/xfeat.pt': '0f5187fd7bedd26c7fe6acc9685444493a165a35ecc087b33c2db3627f3ea10b',
}
OPSET = 17


def space_to_depth(x, ws=2):
    """XFeatModel._unfold2d with pixel_unshuffle: the same tensor (checked in main).

    Upstream slices the 8x8 blocks with Tensor.unfold, which torch.onnx can't export when the
    image size is dynamic ("input size not accessible").
    """
    return F.pixel_unshuffle(x, ws)


class Dense(torch.nn.Module):
    """XFeat's network and the dense part of detectAndCompute."""

    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, image):
        feats, logits, reliability = self.net(image)
        descriptors = F.normalize(feats, dim=1)
        # XFeat.get_kpts_heatmap: softmax over 65 bins, the 64 cells of each 8x8 block.
        scores = F.softmax(logits, 1)[:, :64]
        b, _, h, w = scores.shape
        heatmap = scores.permute(0, 2, 3, 1).reshape(b, h, w, 8, 8)
        heatmap = heatmap.permute(0, 1, 3, 2, 4).reshape(b, 1, h * 8, w * 8)
        # XFeat.NMS: a pixel is kept where it equals the maximum of its 5x5 neighbourhood.
        is_max = heatmap == F.max_pool2d(heatmap, 5, stride=1, padding=2)
        return descriptors, heatmap, reliability, is_max


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--upstream', required=True, type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--parity-images', nargs='*', default=[], type=pathlib.Path)
    parser.add_argument('--top-k', type=int, default=4096)
    args = parser.parse_args()

    common.verify_files(args.upstream, UPSTREAM_FILES)
    sys.path.insert(0, str(args.upstream))
    from modules.model import XFeatModel  # noqa: E402 (upstream, checked above)
    from modules.xfeat import XFeat  # noqa: E402

    torch.manual_seed(0)
    net = XFeatModel().eval()
    net.load_state_dict(common.load_state_dict(args.upstream / 'weights/xfeat.pt'))
    # The network with space_to_depth must give exactly upstream's outputs.
    probe = torch.rand(1, 1, 96, 128)
    with torch.no_grad():
        upstream_outputs = net(probe)
        net._unfold2d = space_to_depth
        patched_outputs = net(probe)
    if not all(torch.equal(a, b) for a, b in zip(upstream_outputs, patched_outputs)):
        raise SystemExit('space_to_depth changes XFeatModel\'s outputs')
    dense = Dense(net).eval()

    args.output.mkdir(parents=True, exist_ok=True)
    model_path = args.output / 'xfeat.onnx'
    example = torch.rand(1, 1, 480, 640)
    torch.onnx.export(
        dense, (example,), str(model_path), opset_version=OPSET,
        input_names=['image'],
        output_names=['descriptors', 'heatmap', 'reliability', 'is_max'],
        dynamic_axes={'image': {2: 'height', 3: 'width'},
                      'descriptors': {2: 'height8', 3: 'width8'},
                      'heatmap': {2: 'height', 3: 'width'},
                      'reliability': {2: 'height8', 3: 'width8'},
                      'is_max': {2: 'height', 3: 'width'}})

    # The graph against PyTorch, at the export size and another one (dynamic shapes).
    import onnxruntime
    session = onnxruntime.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
    for shape in ((480, 640), (352, 512)):
        image = torch.rand(1, 1, *shape)
        with torch.no_grad():
            expected = [t.numpy() for t in dense(image)]
        got = session.run(None, {'image': image.numpy()})
        for name, e, g in zip(('descriptors', 'heatmap', 'reliability'), expected, got):
            error = float(np.max(np.abs(e - g)))
            print(f'{shape}: {name} max abs difference {error:.2e}')
            if error > 1e-4:
                raise SystemExit(f'ONNX and PyTorch differ on {name}')
        mismatch = float(np.mean(expected[3] != got[3]))
        print(f'{shape}: is_max differs at {mismatch:.2e} of the pixels')

    if args.parity_images:
        # Upstream's own detectAndCompute, on the CPU, on [0, 1] grayscale (the C++ input).
        xfeat = XFeat(weights=None, top_k=args.top_k)
        xfeat.dev = torch.device('cpu')
        xfeat.net = net
        writer = common.ParityWriter(args.output / 'xfeat.parity.bin', common.KIND_LOCAL)
        for path in args.parity_images:
            gray = cv2.cvtColor(cv2.imread(str(path), cv2.IMREAD_COLOR), cv2.COLOR_BGR2GRAY)
            tensor = torch.from_numpy(gray).float()[None, None] / 255.0
            out = xfeat.detectAndCompute(tensor, top_k=args.top_k)[0]
            writer.add_local(gray, out['keypoints'].numpy(), out['scores'].numpy(),
                             out['descriptors'].numpy())
            print(f'{path.name}: {out["keypoints"].shape[0]} keypoints')
        writer.close()

    for path in sorted(args.output.glob('xfeat*')):
        print(f'{path.name} {path.stat().st_size} bytes sha256 {common.sha256(path)}')


if __name__ == '__main__':
    main()
