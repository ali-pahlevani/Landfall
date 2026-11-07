# Learned models

Landfall's default backends are classical: ORB features and a bag of words, which need nothing
beyond OpenCV. The learned backends run on ONNX Runtime and are optional: `landfall_core` builds
them when it finds ONNX Runtime (rosdep's `onnxruntime_vendor`, the CPU release), and
`-DLANDFALL_WITH_ONNXRUNTIME=OFF` leaves them out.

> **Status (M5):** the backends work through the builder, the database and the relocalizer. Their
> parity with PyTorch holds on the real models, and they've had a first comparison with the
> classical backends in simulation (below). The exported models go to GitHub release `models-v1`
> (`docs/TODO.md` T-007 until it is published).

## Presets

| Preset | Local features | Global descriptor | Matcher | Status |
|---|---|---|---|---|
| `classical` (default) | `orb` | `bow` | `ratio` | covered by the v1.0 guarantees |
| `learned_cpu` | `xfeat`, 2048 keypoints | `eigenplaces` (ResNet-18, 512-D) | `mnn` | measured in simulation (below); presets are settled by the M7 benchmark |
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

To export them yourself:

```bash
scripts/export_models.sh [OUTPUT_DIR]   # default: the models directory, as above
```

The script works in `~/.cache/landfall/export` (`$LANDFALL_EXPORT_DIR`; about 1.3 GB, which can be
deleted afterwards):
1. It installs the pinned Python packages of `landfall_tools/requirements.txt` in a virtual
   environment.
2. It downloads the pinned upstream files (`scripts/fetch_models.sh --upstream DIR`).
3. It takes four images from the depot fixture bag (`landfall_tools/export/parity_images.py`).
4. It runs the three export scripts of `landfall_tools/export/`, writing the models, their parity
   files and `export.log` (about 520 MB).

Every upstream file is checked against its SHA-256 in `landfall_tools/models.yaml` before it is
used. The exports run upstream code or load downloaded checkpoints: XFeat's model definition, and
the XFeat and EigenPlaces checkpoints through `torch.load(weights_only=True)`. The requirements pin
torch 2.6.0, the first release in which that is not open to CVE-2025-32434. The EigenPlaces network
is built by the script itself from torchvision, since upstream's constructor fetches and runs
CosPlace's code through `torch.hub`. `megaloc.onnx` is used as upstream exported it; its graph
takes ImageNet-normalized RGB at 322 x 322 and has no normalization of its own.

Two functions are written differently for the export, with the same values, so that the graphs take
any image size. XFeat's 8 x 8 unfolding becomes `pixel_unshuffle`; the script checks that the
network's outputs stay bit for bit the same. EigenPlaces' GeM pooling takes a mean instead of
average pooling with a kernel the size of the feature map; the two agree to float rounding.

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
descriptors and global descriptors with a cosine of at least 0.999. The real models' parity files
hold four depot images from the fixture bag, one of them cropped to 600 x 450 so that it's resized;
CI runs them on the published files (Jazzy, amd64). The results below are from Landfall in C++ on
ONNX Runtime 1.24.3 (rosdep's, Jazzy); Ubuntu 26.04's ONNX Runtime 1.23.2 (Lyrical) gives the same.

| Model | Reference | Result |
|---|---|---|
| synthetic XFeat (`test/data`, always run) | XFeat's post-processing with torch's own `grid_sample` and `interpolate` | 65 of 65 keypoints on three images, two of them resized; descriptor cosine 1.0 |
| synthetic global descriptor (always run) | the preprocessing in NumPy | cosine 1.0 |
| `xfeat.onnx` | upstream's `XFeat.detectAndCompute` in PyTorch 2.6, 4096 keypoints | 4096, 4095, 4095 and 4096 of 4096 keypoints within 0.5 px (the last image is the resized one); descriptor cosine 1.00000 at worst |
| `eigenplaces_resnet18_512.onnx` | the network in PyTorch 2.6 | cosine 1.00000 on all four images |
| `megaloc.onnx` | upstream's file in ONNX Runtime 1.20.1 (Python) | cosine 1.00000 on all four images |

The synthetic XFeat's heatmap carries a tiny ramp (1e-4 per column, 1e-5 per row). Without it,
every plateau of an 8-bit image is a set of exact ties. OpenCV's and PyTorch's bilinear resizes
round differently in the last bit, and that alone picked different maxima on resized images. On
images that need no resize, parity was exact even with the ties: 5391 of 5391 keypoints. Real
XFeat heatmaps come out of a softmax, where exact ties don't occur.

## Learned and classical backends in simulation (M5)

The depot drives and the fixture of `docs/relocalization.md`, queried the same way: each drive's
images, one per second, against a database of the other drive built with the same backends, with
the default thresholds and the LiDAR check. Both directions are added up; the error and time
columns give the range over the two. Success means within 0.25 m and 5°; wrong means more than
0.5 m or 10° off.

| Backends | Request | Success | AMBIGUOUS | NO_MATCH | REJECTED_CONSISTENCY | Wrong poses | Error, median / p95 | Time per frame (median) |
|---|---|---|---|---|---|---|---|---|
| `classical`: ORB, bag of words | 1 frame | 859 / 871 (98.6 %) | 10 | 0 | 0 | 2 | 1.2–1.5 / 3.6–4.1 cm | 31–32 ms |
| | 3 frames | 282 / 295 (95.6 %) | 10 | 1 | 2 | 0 | 1.1–1.3 / 2.5–3.5 cm | |
| `learned_cpu`: XFeat, EigenPlaces | 1 frame | 669 / 694 (96.4 %) | 7 | 18 | 0 | 0 | 1.8–2.0 / 6.2–6.4 cm | 143–145 ms |
| | 3 frames | 218 / 247 (88.3 %) | 7 | 21 | 1 | 0 | 1.5–1.7 / 4.4–4.5 cm | |
| ORB, EigenPlaces | 1 frame | 869 / 876 (99.2 %) | 5 | 0 | 0 | 2 | 1.3–1.5 / 3.6–4.3 cm | 68–71 ms |
| | 3 frames | 287 / 296 (97.0 %) | 5 | 2 | 2 | 0 | 1.1–1.4 / 2.9–4.1 cm | |

On the fixture's non-keyframe images (one frame per request), `classical` and ORB with EigenPlaces
succeeded on 25 of 27 requests and `learned_cpu` on 30 of 31, none of them wrong.

No 3-frame request published a wrong pose with any of the backends. The single-frame wrong poses
are the depot's mirror image (`docs/relocalization.md`); XFeat avoided them, at fewer successes.
`learned_cpu` misses more because its maps are thinner. From the same keyframes it triangulates
8,662 landmarks against ORB's 14,446, with shorter tracks (3.55 against 3.97 views), so its
successful single-frame requests have fewer inliers (median 85 against 124). Where a stretch of
the drive falls below the 40 inliers required (`sanity.min_inliers`), a request answers NO_MATCH.
EigenPlaces retrieval with ORB features did best: the most successes and half the ambiguous
frames, at about twice the classical time.

These are clean renders of one simulated building, timed on the development laptop (i7-13620H).
Building run 1's database took 84 s with `classical`, 125 s with `learned_cpu` and 80 s with ORB
and EigenPlaces, for 21.3, 17.1 and 10.9 MB. The benchmark (M7) and the field validation (M8)
settle the presets and their tuning, such as XFeat's keypoint count.

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
