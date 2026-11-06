# Learned models

Landfall's default backends are classical: ORB features and a bag of words, which need nothing
beyond OpenCV. The learned backends run on ONNX Runtime and are optional: `landfall_core` builds
them when it finds ONNX Runtime (rosdep's `onnxruntime_vendor`, the CPU release), and
`-DLANDFALL_WITH_ONNXRUNTIME=OFF` leaves them out.

> **Status (M5):** the backends, their plumbing through the builder, the database and the
> relocalizer, and their parity with PyTorch are in place and tested on synthetic models. The real
> models are not published yet (`docs/TODO.md` T-007), so nothing learned has been measured on
> images yet.

## Presets

| Preset | Local features | Global descriptor | Matcher | Status |
|---|---|---|---|---|
| `classical` (default) | `orb` | `bow` | `ratio` | covered by the v1.0 guarantees |
| `learned_cpu` | `xfeat`, 2048 keypoints | `eigenplaces` (ResNet-18, 512-D) | `mnn` | models pending (T-007) |
| `learned_gpu` (experimental) | `xfeat`, 4096 keypoints | `megaloc` (8448-D) | `lighterglue` (T-008) | `mnn` until LighterGlue lands |

ALIKED, the plan's first choice for the GPU preset, can't run on ONNX Runtime: neither 1.23.2 nor
1.24.3 has a kernel for the `DeformConv` operator its deformable convolutions need. The plan's
fallback, XFeat with LighterGlue, takes its place (`docs/PLAN.md` §10).

## Using them

Choose them in the build configuration:

```yaml
backend: {local: xfeat, global: eigenplaces, models_dir: /opt/landfall/models}
features: {max_keypoints: 2048}  # xfeat options: model, max_keypoints, detection_threshold
global: {}                       # eigenplaces / megaloc options: model, width, height
```

`landfall build` and `landfall query` take `--models-dir` and `--execution-provider`; the
relocalizer takes the parameters `backend.models_dir` and `backend.execution_provider`. Without a
directory, models are looked up in `$LANDFALL_MODELS_DIR`, else `~/.local/share/landfall/models`.

A database records the SHA-256 of every model it was built with. The relocalizer refuses to load
it with a missing model or a different one under the same file name, and says which. A build
without ONNX Runtime refuses a database built with learned backends.

`execution_provider: cuda` or `tensorrt` needs an ONNX Runtime built with them, given as
`ONNXRUNTIME_ROOT` when building Landfall, and NVIDIA's CUDA, cuDNN or TensorRT libraries (closed
source). Neither is part of the release guarantees (`docs/PLAN.md` D11).

## Getting the models

```bash
scripts/fetch_models.sh                 # all of landfall_tools/models.yaml
scripts/fetch_models.sh --parity        # with the parity references of the C++ tests
```

Every file is checked against its SHA-256 before it is installed. `megaloc.onnx` is upstream's own
file, fetched from its pinned Hugging Face revision; the others come from Landfall's `models-v1`
release once T-007 is done.

To export them yourself, in a virtual environment:

```bash
python3 -m venv .venv && .venv/bin/pip install -r landfall_tools/requirements.txt
.venv/bin/python -I landfall_tools/export/xfeat.py --upstream ACCELERATED_FEATURES --output OUT \
  --parity-images IMAGE...
.venv/bin/python -I landfall_tools/export/eigenplaces.py --weights ResNet18_512_eigenplaces.pth \
  --output OUT --parity-images IMAGE...
.venv/bin/python -I landfall_tools/export/megaloc.py --onnx megaloc.onnx --output OUT \
  --parity-images IMAGE...
```

Each script checks the SHA-256 of the upstream files it uses (`landfall_tools/models.yaml`). They
run upstream code or load downloaded checkpoints: XFeat's model definition, and the XFeat and
EigenPlaces checkpoints through `torch.load(weights_only=True)`. The requirements pin torch 2.6.0,
the first release in which that is not open to CVE-2025-32434. The EigenPlaces network is built by
the script itself from torchvision, since upstream's constructor fetches and runs CosPlace's code
through `torch.hub`.

## What the models compute

**xfeat.onnx**: XFeat's network and the dense steps of its `detectAndCompute`. Input `image`,
float32 `[1, 1, H, W]`, grayscale in [0, 1], H and W multiples of 32. Outputs: `descriptors`
`[1, 64, H/8, W/8]` (L2-normalized), `heatmap` `[1, 1, H, W]`, `reliability` `[1, 1, H/8, W/8]`,
and `is_max` (bool, the 5x5 local maxima of the heatmap). The rest runs in C++
(`landfall_core/src/onnx/xfeat.cpp`), step by step like XFeat: the threshold (0.05), scores
(heatmap times reliability), the best keypoints, and descriptors sampled bicubically, all with
PyTorch's `grid_sample` arithmetic. Images whose sides aren't multiples of 32 are resized first,
as XFeat does.

**eigenplaces, megaloc**: one image in, one L2-normalized vector out. Landfall feeds the color
image as RGB in [0, 1] with ImageNet's mean and standard deviation removed, at its own size
(EigenPlaces) or resized to 322 x 322 with `cv::INTER_AREA` (MegaLoc; upstream evaluates with an
antialiased resize).

## Parity with PyTorch

`test_onnx_parity` runs each backend on the images of a parity file and compares with what
PyTorch or ONNX Runtime computed in Python. The bars: keypoints within 0.5 px for at least 95 %,
descriptors and global descriptors with a cosine of at least 0.999.

| Model | Reference | Result |
|---|---|---|
| synthetic XFeat (`test/data`, always run) | XFeat's post-processing with torch's own `grid_sample` and `interpolate` | 65 of 65 keypoints on three images, two of them resized; descriptor cosine 1.0 |
| synthetic global descriptor (always run) | the preprocessing in NumPy | cosine 1.0 |
| xfeat.onnx, eigenplaces, megaloc | upstream's PyTorch | pending (T-007): skipped while their files are missing |

The synthetic XFeat's heatmap carries a tiny ramp (1e-4 per column, 1e-5 per row). Without it,
every plateau of an 8-bit image is a set of exact ties. OpenCV's and PyTorch's bilinear resizes
round differently in the last bit, and that alone picked different maxima on resized images. On
images that need no resize, parity was exact even with the ties: 5391 of 5391 keypoints. Real
XFeat heatmaps come out of a softmax, where exact ties don't occur.

## Licenses and provenance

Only permissively licensed models are used; their weights are downloaded, never committed.

| Model | Code | Weights | Training data | Notes |
|---|---|---|---|---|
| XFeat | Apache-2.0 | Apache-2.0, published with the code | MegaDepth and COCO images (photos under their own terms) | |
| EigenPlaces | MIT | MIT, release v1.0 | SF-XL street-level imagery of San Francisco | backbone: torchvision ResNet-18 (BSD-3-Clause) |
| MegaLoc | MIT | MIT | undisclosed | DINOv2 backbone (Apache-2.0); upstream's own ONNX export |
| LighterGlue | Apache-2.0 | Apache-2.0, in XFeat's repository | see verlab/accelerated_features | T-008 |

Not used: SuperPoint and SuperGlue (non-commercial), SALAD (GPL-3.0), NetVLAD and MixVPR (no
license found upstream), DINOv3-based models (DINOv3's own license, not OSI-approved).
`docs/PLAN.md` §10 has the full shortlist.
