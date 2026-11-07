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

"""Export EigenPlaces (ResNet-18, 512-D) to ONNX for landfall_core's `eigenplaces` backend.

EigenPlaces: gmberton/EigenPlaces (MIT), weights from its v1.0 release, pinned below.

    python3 -I eigenplaces.py --weights ResNet18_512_eigenplaces.pth --output DIR
                              [--parity-images IMAGE ...]

The network is built here, not imported: upstream's constructor fetches and runs CosPlace's
code from torch.hub only to initialize weights that the EigenPlaces checkpoint then replaces.
It is torchvision's ResNet-18 without its pooling and classifier, followed by EigenPlaces'
aggregation (L2 norm over channels, GeM pooling with a learned p, a linear layer, L2 norm),
eigenplaces_model/eigenplaces_network.py at UPSTREAM_COMMIT. load_state_dict(strict=True) fails
on any difference in the architecture.

    input  image       float32 [1, 3, H, W]   RGB in [0, 1], ImageNet mean and std removed
    output descriptor  float32 [1, 512]       L2-normalized

With --parity-images, eigenplaces.parity.bin holds the images (BGR) and the descriptors this
network computes on them, preprocessed as landfall_core's OnnxGlobalDescriptor does (see
common.py).
"""

import argparse
import pathlib
import sys

import cv2
import numpy as np
import torch
import torch.nn.functional as F
import torchvision

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # python3 -I drops it
import common  # noqa: E402

UPSTREAM_COMMIT = 'a2969f71d5ea31017443af490b15273ca4c50af1'
WEIGHTS_URL = ('https://github.com/gmberton/EigenPlaces/releases/download/v1.0/'
               'ResNet18_512_eigenplaces.pth')
WEIGHTS_SHA256 = 'b47ea1efb51041604e9c135508764175bbaa98dece47b47da664517317d0aff3'
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
OPSET = 17


class GeM(torch.nn.Module):
    """Generalized mean pooling over the whole feature map.

    Upstream averages with F.avg_pool2d and a kernel the size of the feature map. That kernel is
    an attribute in ONNX and can't follow the image size, so the export fails with a dynamic
    input. The same mean is taken with mean() here; the parameter is the same.
    """

    def __init__(self):
        super().__init__()
        self.p = torch.nn.Parameter(torch.ones(1) * 3)
        self.eps = 1e-6

    def forward(self, x):
        pooled = x.clamp(min=self.eps).pow(self.p).mean(dim=(-2, -1), keepdim=True)
        return pooled.pow(1.0 / self.p)


class L2Norm(torch.nn.Module):
    def forward(self, x):
        return F.normalize(x, p=2.0, dim=1)


class Flatten(torch.nn.Module):
    def forward(self, x):
        return x[:, :, 0, 0]


class EigenPlaces(torch.nn.Module):
    """GeoLocalizationNet_('ResNet18', 512) of EigenPlaces, without the torch.hub detour."""

    def __init__(self):
        super().__init__()
        resnet = torchvision.models.resnet18()
        self.backbone = torch.nn.Sequential(*list(resnet.children())[:-2])
        self.aggregation = torch.nn.Sequential(
            L2Norm(), GeM(), Flatten(), torch.nn.Linear(512, 512), L2Norm())

    def forward(self, x):
        return self.aggregation(self.backbone(x))


def preprocess(bgr):
    """OnnxGlobalDescriptor's input: RGB, [0, 1], ImageNet normalization, NCHW."""
    rgb = bgr[:, :, ::-1].astype(np.float32) / 255.0
    return torch.from_numpy(((rgb - MEAN) / STD).transpose(2, 0, 1).copy())[None]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--weights', required=True, type=pathlib.Path)
    parser.add_argument('--output', required=True, type=pathlib.Path)
    parser.add_argument('--parity-images', nargs='*', default=[], type=pathlib.Path)
    args = parser.parse_args()

    if common.sha256(args.weights) != WEIGHTS_SHA256:
        raise SystemExit(f'{args.weights}: not the pinned {WEIGHTS_URL}')
    net = EigenPlaces().eval()
    net.load_state_dict(common.load_state_dict(args.weights), strict=True)

    args.output.mkdir(parents=True, exist_ok=True)
    model_path = args.output / 'eigenplaces_resnet18_512.onnx'
    torch.onnx.export(
        net, (torch.rand(1, 3, 480, 640),), str(model_path), opset_version=OPSET,
        input_names=['image'], output_names=['descriptor'],
        dynamic_axes={'image': {2: 'height', 3: 'width'}})

    import onnxruntime
    session = onnxruntime.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])
    for shape in ((480, 640), (240, 320)):
        image = torch.rand(1, 3, *shape)
        with torch.no_grad():
            expected = net(image).numpy()
        got = session.run(None, {'image': image.numpy()})[0]
        cosine = float(np.sum(expected * got))
        print(f'{shape}: ONNX vs PyTorch cosine {cosine:.6f}')
        if cosine < 0.9999:
            raise SystemExit('ONNX and PyTorch differ')

    if args.parity_images:
        writer = common.ParityWriter(args.output / 'eigenplaces.parity.bin', common.KIND_GLOBAL)
        for path in args.parity_images:
            bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
            with torch.no_grad():
                descriptor = net(preprocess(bgr))[0].numpy()
            writer.add_global(bgr, descriptor)
            print(f'{path.name}: descriptor of {descriptor.shape[0]}')
        writer.close()

    for path in sorted(args.output.glob('eigenplaces*')):
        print(f'{path.name} {path.stat().st_size} bytes sha256 {common.sha256(path)}')


if __name__ == '__main__':
    main()
