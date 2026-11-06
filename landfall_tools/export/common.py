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

"""Helpers shared by the export scripts: checksums of upstream files and the parity file.

The parity file holds test images and what the upstream PyTorch model computes on them; the
C++ parity tests (landfall_core/test/test_onnx_parity.cpp) run the ONNX backend on the same
pixels and compare. Little-endian layout, version 1:

    magic   8 bytes  b"LFPARITY"
    version u32      1
    kind    u32      1: local features, 2: global descriptor
    count   u32      images
    per image:
      height u32, width u32, channels u32 (1: gray, 3: BGR), pixels u8[height*width*channels]
      kind 1: n u32, dim u32, keypoints f32[n*2] (x, y), scores f32[n], descriptors f32[n*dim]
      kind 2: dim u32, descriptor f32[dim]
"""

import hashlib
import pathlib
import struct

import numpy as np

KIND_LOCAL = 1
KIND_GLOBAL = 2


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_files(root, expected):
    """Check that root/<path> has the given SHA-256 for every entry of `expected`."""
    root = pathlib.Path(root)
    for relative, digest in expected.items():
        path = root / relative
        if not path.is_file():
            raise SystemExit(f'missing upstream file {path}')
        actual = sha256(path)
        if actual != digest:
            raise SystemExit(f'{path}: SHA-256 {actual}, expected {digest}')


def load_state_dict(path):
    """Weights only: torch.load without unpickling arbitrary objects."""
    import torch
    return torch.load(path, map_location='cpu', weights_only=True)


class ParityWriter:
    """Writes the parity file described in the module docstring."""

    def __init__(self, path, kind):
        self._file = open(path, 'wb')
        self._kind = kind
        self._count = 0
        self._file.write(b'LFPARITY')
        self._file.write(struct.pack('<III', 1, kind, 0))  # count is patched on close

    def _image(self, pixels):
        pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
        channels = 1 if pixels.ndim == 2 else pixels.shape[2]
        self._file.write(struct.pack('<III', pixels.shape[0], pixels.shape[1], channels))
        self._file.write(pixels.tobytes())
        self._count += 1

    def add_local(self, pixels, keypoints, scores, descriptors):
        assert self._kind == KIND_LOCAL
        self._image(pixels)
        keypoints = np.ascontiguousarray(keypoints, dtype='<f4')
        scores = np.ascontiguousarray(scores, dtype='<f4')
        descriptors = np.ascontiguousarray(descriptors, dtype='<f4')
        self._file.write(struct.pack('<II', keypoints.shape[0], descriptors.shape[1]))
        for array in (keypoints, scores, descriptors):
            self._file.write(array.tobytes())

    def add_global(self, pixels, descriptor):
        assert self._kind == KIND_GLOBAL
        self._image(pixels)
        descriptor = np.ascontiguousarray(descriptor, dtype='<f4').reshape(-1)
        self._file.write(struct.pack('<I', descriptor.shape[0]))
        self._file.write(descriptor.tobytes())

    def close(self):
        self._file.seek(16)
        self._file.write(struct.pack('<I', self._count))
        self._file.close()
