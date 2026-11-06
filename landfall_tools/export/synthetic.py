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

"""Write the tiny synthetic ONNX models of landfall_core's tests (test/data/).

    python3 -I synthetic.py OUTPUT_DIR

No trained weights: graphs whose outputs the tests can predict, with the input and output
contracts of the real models (xfeat.py, the global descriptors), so the ONNX Runtime plumbing and
the post-processing are tested without downloading anything.

synthetic_xfeat.onnx:   image [1, 1, H, W] (H, W multiples of 32)
    heatmap      = image + 1e-4 * column + 1e-5 * row: distinct values in every 5x5 window, so
                   plateaus of the 8-bit image don't make maxima of exact ties
    is_max       = heatmap == maxpool5x5(heatmap)
    reliability  = 8x8 average of the image
    descriptors  = L2-normalized over 64 channels of w_c * reliability + b_c,
                   w_c = cos(c), b_c = sin(c) / 4
synthetic_global.onnx:  image [1, 3, H, W] -> descriptor [1, 3], the per-channel mean
synthetic_global_other.onnx: the same graph, other bytes (a model the database wasn't built with)

and their parity references (common.py), from PyTorch and ONNX Runtime in Python, on procedural
images (sums of Gaussian blobs, 150 x 110: not multiples of 32):
synthetic_xfeat.parity.bin:  XFeat.detectAndCompute's steps after the network, transcribed below
    from verlab/accelerated_features (Apache-2.0) with torch's own grid_sample and interpolate, on
    the synthetic model's outputs: the reference for landfall_core's C++ version of them;
synthetic_global.parity.bin: the descriptors of OnnxGlobalDescriptor's preprocessing done with
    OpenCV and numpy.
"""

import math
import pathlib
import sys

import cv2
import numpy as np
import onnx
from onnx import TensorProto, helper
import torch
import torch.nn.functional as F

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import common  # noqa: E402

OPSET = 17
PARITY_TOP_K = 100000  # every keypoint: ties in the scores can't pick different ones


def xfeat_model():
    image = helper.make_tensor_value_info('image', TensorProto.FLOAT, [1, 1, 'height', 'width'])
    weights = np.array([math.cos(c) for c in range(64)], np.float32).reshape(64, 1, 1, 1)
    bias = np.array([math.sin(c) / 4.0 for c in range(64)], np.float32)
    nodes = [
        # The ramps: column and row indices from the input's shape.
        helper.make_node('Shape', ['image'], ['shape']),
        helper.make_node('Gather', ['shape', 'axis_w'], ['width_scalar'], axis=0),
        helper.make_node('Gather', ['shape', 'axis_h'], ['height_scalar'], axis=0),
        helper.make_node('Range', ['zero', 'width_scalar', 'one'], ['columns']),
        helper.make_node('Range', ['zero', 'height_scalar', 'one'], ['rows']),
        helper.make_node('Cast', ['columns'], ['columns_f'], to=TensorProto.FLOAT),
        helper.make_node('Cast', ['rows'], ['rows_f'], to=TensorProto.FLOAT),
        helper.make_node('Mul', ['columns_f', 'column_step'], ['column_ramp']),
        helper.make_node('Mul', ['rows_f', 'row_step'], ['row_ramp']),
        helper.make_node('Reshape', ['column_ramp', 'row_shape'], ['column_ramp_4d']),
        helper.make_node('Reshape', ['row_ramp', 'column_shape'], ['row_ramp_4d']),
        helper.make_node('Add', ['image', 'column_ramp_4d'], ['image_x']),
        helper.make_node('Add', ['image_x', 'row_ramp_4d'], ['heatmap']),
        helper.make_node('MaxPool', ['heatmap'], ['pooled'], kernel_shape=[5, 5],
                         strides=[1, 1], pads=[2, 2, 2, 2]),
        helper.make_node('Equal', ['heatmap', 'pooled'], ['is_max']),
        helper.make_node('AveragePool', ['image'], ['reliability'], kernel_shape=[8, 8],
                         strides=[8, 8]),
        helper.make_node('Conv', ['reliability', 'weights', 'bias'], ['raw']),
        helper.make_node('ReduceL2', ['raw'], ['norm'], axes=[1], keepdims=1),
        helper.make_node('Max', ['norm', 'eps'], ['safe_norm']),
        helper.make_node('Div', ['raw', 'safe_norm'], ['descriptors']),
    ]
    initializers = [
        helper.make_tensor('axis_w', TensorProto.INT64, [], [3]),
        helper.make_tensor('axis_h', TensorProto.INT64, [], [2]),
        helper.make_tensor('zero', TensorProto.INT64, [], [0]),
        helper.make_tensor('one', TensorProto.INT64, [], [1]),
        helper.make_tensor('column_step', TensorProto.FLOAT, [], [1e-4]),
        helper.make_tensor('row_step', TensorProto.FLOAT, [], [1e-5]),
        helper.make_tensor('row_shape', TensorProto.INT64, [4], [1, 1, 1, -1]),
        helper.make_tensor('column_shape', TensorProto.INT64, [4], [1, 1, -1, 1]),
        helper.make_tensor('weights', TensorProto.FLOAT, weights.shape, weights.flatten()),
        helper.make_tensor('bias', TensorProto.FLOAT, bias.shape, bias),
        helper.make_tensor('eps', TensorProto.FLOAT, [], [1e-12]),
    ]
    outputs = [
        helper.make_tensor_value_info('descriptors', TensorProto.FLOAT,
                                      [1, 64, 'height8', 'width8']),
        helper.make_tensor_value_info('heatmap', TensorProto.FLOAT, [1, 1, 'height', 'width']),
        helper.make_tensor_value_info('reliability', TensorProto.FLOAT,
                                      [1, 1, 'height8', 'width8']),
        helper.make_tensor_value_info('is_max', TensorProto.BOOL, [1, 1, 'height', 'width']),
    ]
    graph = helper.make_graph(nodes, 'synthetic_xfeat', [image], outputs, initializers)
    return helper.make_model(graph, opset_imports=[helper.make_opsetid('', OPSET)],
                             producer_name='landfall_tools/export/synthetic.py')


def global_model(doc=''):
    image = helper.make_tensor_value_info('image', TensorProto.FLOAT, [1, 3, 'height', 'width'])
    nodes = [
        helper.make_node('GlobalAveragePool', ['image'], ['pooled']),
        helper.make_node('Flatten', ['pooled'], ['descriptor'], axis=1),
    ]
    output = helper.make_tensor_value_info('descriptor', TensorProto.FLOAT, [1, 3])
    graph = helper.make_graph(nodes, 'synthetic_global', [image], [output], doc_string=doc)
    return helper.make_model(graph, opset_imports=[helper.make_opsetid('', OPSET)],
                             producer_name='landfall_tools/export/synthetic.py')


def blobs(seed, channels, rows=110, cols=150):
    """A smooth procedural image: a gradient and Gaussian blobs."""
    rng = np.random.default_rng(seed)
    ys, xs = np.mgrid[0:rows, 0:cols].astype(np.float64)
    planes = []
    for _ in range(channels):
        plane = 30.0 + 0.2 * xs + 0.1 * ys
        for _ in range(25):
            cx, cy = rng.uniform(0, cols), rng.uniform(0, rows)
            sigma, height = rng.uniform(1.5, 4.0), rng.uniform(40.0, 180.0)
            plane += height * np.exp(-((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * sigma ** 2))
        planes.append(plane)
    image = np.clip(np.stack(planes, axis=-1), 0, 255).round().astype(np.uint8)
    return image[:, :, 0] if channels == 1 else image


class InterpolateSparse2d(torch.nn.Module):
    """verlab/accelerated_features modules/interpolator.py (Apache-2.0)."""

    def __init__(self, mode):
        super().__init__()
        self.mode = mode

    def normgrid(self, x, H, W):
        return 2. * (x / (torch.tensor([W - 1, H - 1], device=x.device, dtype=x.dtype))) - 1.

    def forward(self, x, pos, H, W):
        grid = self.normgrid(pos, H, W).unsqueeze(-2).to(x.dtype)
        x = F.grid_sample(x, grid, mode=self.mode, align_corners=False)
        return x.permute(0, 2, 3, 1).squeeze(-2)


def xfeat_reference(session, gray, top_k, threshold=0.05):
    """XFeat.detectAndCompute after the network (modules/xfeat.py, Apache-2.0), on the outputs
    of an ONNX model with xfeat.py's contract."""
    x = torch.from_numpy(gray).float()[None, None] / 255.0
    H, W = x.shape[-2:]
    _H, _W = (H // 32) * 32, (W // 32) * 32
    rh, rw = H / _H, W / _W
    x = F.interpolate(x, (_H, _W), mode='bilinear', align_corners=False)
    descriptors, heatmap, reliability, is_max = (
        torch.from_numpy(o) for o in session.run(None, {'image': x.numpy()}))
    pos = is_max & (heatmap > threshold)
    mkpts = pos[0].nonzero()[..., 1:].flip(-1)[None]
    scores = (InterpolateSparse2d('nearest')(heatmap, mkpts, _H, _W) *
              InterpolateSparse2d('bilinear')(reliability, mkpts, _H, _W)).squeeze(-1)
    scores[torch.all(mkpts == 0, dim=-1)] = -1
    idxs = torch.argsort(-scores)
    mkpts_x = torch.gather(mkpts[..., 0], -1, idxs)[:, :top_k]
    mkpts_y = torch.gather(mkpts[..., 1], -1, idxs)[:, :top_k]
    mkpts = torch.cat([mkpts_x[..., None], mkpts_y[..., None]], dim=-1)
    scores = torch.gather(scores, -1, idxs)[:, :top_k]
    feats = InterpolateSparse2d('bicubic')(descriptors, mkpts, H=_H, W=_W)
    feats = F.normalize(feats, dim=-1)
    mkpts = mkpts * torch.tensor([rw, rh]).view(1, 1, -1)
    valid = scores > 0
    return mkpts[0][valid[0]].numpy(), scores[0][valid[0]].numpy(), feats[0][valid[0]].numpy()


def global_reference(session, bgr):
    """OnnxGlobalDescriptor's preprocessing (eigenplaces preset: the image's own size)."""
    rgb = bgr[:, :, ::-1].astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], np.float32)
    std = np.array([0.229, 0.224, 0.225], np.float32)
    x = ((rgb - mean) / std).transpose(2, 0, 1)[None].astype(np.float32)
    d = session.run(None, {'image': x})[0][0]
    return d / np.linalg.norm(d)


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: synthetic.py OUTPUT_DIR')
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for name, model in (('synthetic_xfeat.onnx', xfeat_model()),
                        ('synthetic_global.onnx', global_model()),
                        ('synthetic_global_other.onnx', global_model('another model'))):
        # IR version 8 (ONNX 1.10+) loads in every ONNX Runtime the backends support.
        model.ir_version = 8
        onnx.checker.check_model(model)
        onnx.save(model, str(out / name))
        print(f'{name}: {(out / name).stat().st_size} bytes')

    import onnxruntime
    xfeat = onnxruntime.InferenceSession(str(out / 'synthetic_xfeat.onnx'),
                                         providers=['CPUExecutionProvider'])
    writer = common.ParityWriter(out / 'synthetic_xfeat.parity.bin', common.KIND_LOCAL)
    for seed, rows, cols in ((1, 110, 150), (2, 110, 150), (5, 128, 160)):
        gray = blobs(seed, 1, rows, cols)
        keypoints, scores, descriptors = xfeat_reference(xfeat, gray, PARITY_TOP_K)
        writer.add_local(gray, keypoints, scores, descriptors)
        print(f'synthetic_xfeat image {seed} ({cols} x {rows}): {len(keypoints)} keypoints')
    writer.close()
    glob = onnxruntime.InferenceSession(str(out / 'synthetic_global.onnx'),
                                        providers=['CPUExecutionProvider'])
    writer = common.ParityWriter(out / 'synthetic_global.parity.bin', common.KIND_GLOBAL)
    for seed in (3, 4):
        bgr = cv2.cvtColor(blobs(seed, 3), cv2.COLOR_RGB2BGR)
        writer.add_global(bgr, global_reference(glob, bgr))
    writer.close()
    for name in ('synthetic_xfeat.parity.bin', 'synthetic_global.parity.bin'):
        print(f'{name}: {(out / name).stat().st_size} bytes')


if __name__ == '__main__':
    main()
