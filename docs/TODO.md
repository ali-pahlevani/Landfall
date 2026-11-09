# TODO register

Nothing is stubbed silently. Every open item in code is written as `TODO(T-NNN): …` and must have a
row here. `scripts/check_todos.sh` enforces this in the lint workflow. Remove a row when its item
is done.

| ID | Where | What | Context | Planned |
|---|---|---|---|---|
| T-004 | `landfall_nav2/package.xml` | Remove the Lyrical-only `nav2_common` build dependency | Nav2 1.5.2 (Lyrical) exports `nav2_common` from `nav2_behavior_tree`'s CMake config, but `ros-lyrical-nav2-behavior-tree` doesn't depend on `ros-lyrical-nav2-common`, so `find_package(nav2_behavior_tree)` fails on a clean system (found in M0) | when fixed upstream |
| T-006 | `landfall_core_build` BA | Optional extrinsics refinement (SO(3) quaternion manifold, off by default; PLAN §7.6) | No consumer yet: the M2 recordings need no extrinsics correction. Reconsider with field data | after M8 |
| T-008 | `landfall_core` ONNX backends | LighterGlue matcher (XFeat's LightGlue), for the experimental GPU preset | ALIKED cannot run on ONNX Runtime (no `DeformConv` kernel in 1.23.2 or 1.24.3), so the GPU preset falls back to XFeat + LighterGlue + MegaLoc (PLAN §10); its ONNX export needs XFeat's `lighterglue.py` (kornia) and an adapted LightGlue-ONNX | M7 |
| T-009 | `landfall_core/src/onnx/session.cpp` | Verify the CUDA and TensorRT execution providers | Unverified: the development machine has CUDA 12.6 but no cuDNN or TensorRT, and rosdep's ONNX Runtime is CPU-only; a provider it lacks is refused with a clear error (tested). Needs an ONNX Runtime built with them (`ONNXRUNTIME_ROOT`). Outside the v1.0 guarantees (PLAN D11) | when cuDNN is available |
| T-010 | `.github/workflows/nightly.yml` (missing) | Nightly simulation smoke test on Jazzy, allowed to fail (PLAN §12): a few kidnap placements against the fixture's database | GitHub's runners have no GPU, so Gazebo would render the camera and the LiDAR in software (Mesa llvmpipe); whether that runs fast enough is untested. Testing it here needs the machine idle, and the M7 benchmark had it busy | M7 part 2 or 1.x |
