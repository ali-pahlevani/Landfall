# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) (see `docs/PLAN.md` §11.1).

## [Unreleased]

### Added
- Implementation plan (`docs/PLAN.md`).
- Package skeletons: `landfall_core`, `landfall_msgs`, `landfall_ros`, `landfall_nav2`,
  `landfall_bringup`, `landfall_benchmark`, plus `landfall_tools` (Python, not a ROS package).
- Interface definitions in `landfall_msgs`: `Relocalize.srv`, `RelocalizationResult`,
  `RelocalizationScores`, `Hypothesis`, `RecorderStatus` (not frozen until v1.0.0-rc.1).
- Distro compatibility layer with compiled probes: Ceres 2.0/2.2 manifolds, `cv_bridge` headers,
  rosbag2 receive timestamps, bond and diagnostics on plain lifecycle nodes, Nav2 `BtServiceNode`
  with BehaviorTree.CPP v3/v4, and CMake helpers for packages without exported targets.
- Docker image for Humble, Jazzy and Lyrical; GitHub Actions for build/test and lint.
- `landfall_core` library (M1): camera models (plumb_bob, rational_polynomial, equidistant),
  SHA-256, SQLite map database schema v1 with migrations, map_server-compatible map loader (Humble
  and Jazzy pixel rules), likelihood-field LiDAR scan verifier, ORB/SIFT with grid bucketing,
  ratio and mutual-nearest matchers, vocabulary tree with TF-IDF bag of words and inverted index,
  dense retrieval index, upright 2-point / P3P / OpenCV PnP (PoseLib LO-RANSAC), gravity from IMU
  or flat floor, sanity checks, pose clustering and ambiguity, multi-frame odometry consistency,
  decision policy, extrinsics health estimator.
- `landfall_core_build` library: known-pose triangulation (pair RANSAC, multi-view DLT, Ceres
  refinement, filters).
- AddressSanitizer + UndefinedBehaviorSanitizer CI job for `landfall_core`.
- Keyframe recording (M2): `landfall_recorder` lifecycle node and component (keyframe gates on
  localization covariance, motion, speed, IMU tilt, sharpness and exposure; crash-safe staging
  folders; status, markers, pause service, diagnostics, bond), and the same gates on recorded bags.
- Map building (M2): `landfall build`, `inspect`, `report`, `verify` and `migrate`. The builder
  extracts features, trains the vocabulary, selects and verifies image pairs, builds tracks,
  triangulates with the poses fixed, runs the bounded pose-prior bundle adjustment, and writes a
  deterministic database with a report (`report.md`, `report.json`, `coverage.png`).
- Backend registry, thread pool, two-view geometry (5-point and rotation-only LO-RANSAC), database
  codecs for features and global descriptors.
- Depot simulation (Gazebo Harmonic, Nav2, TurtleBot 4 with a 640 x 480 camera and ground truth)
  for recording and evaluation: `docker/Dockerfile.sim`, `scripts/sim.sh`, `landfall_bringup/sim/`;
  `evaluate_poses` compares recorded and built poses with the ground truth.
- Test fixture `depot_small` (4.9 MB bag readable on every distro) and CI tests that build it and
  compare the map loader with `nav2_map_server`.
- Relocalization (M3): `landfall_relocalizer` lifecycle node and component. It answers
  `~/relocalize` on a worker thread (deferred response), checks readiness first (database, map
  match, camera, AMCL active, a subscriber, TF), and collects frames until enough agree with
  odometry. Each frame goes through retrieval, places, 2D-3D matching, the upright or P3P pose and
  the sanity checks, and frames that look like two places are ambiguous. The pose is
  motion-compensated before it goes to `/initialpose` or AMCL's `set_initial_pose`. The node also
  has a startup trigger, `~/status`, markers, a debug image, `debug.dump_dir`, diagnostics, a bond,
  the startup extrinsics check and IMU gravity. Thresholds can change at runtime.
- `landfall_core`: `LocalizationMap` (read-only database with lazily loaded, cached keyframe
  features) and the single-frame `Localizer`.
- `landfall query`: relocalizes a bag offline, emulating node requests, and writes one CSV row per
  request. `evaluate_poses --queries` scores the rows against ground truth.
- `docs/parameters.md`, generated from the nodes' declarations (`scripts/update_parameters_doc.sh`).
  A test fails when it is stale.
- `docs/relocalization.md`, and the M3 simulation results: 86 % success with 3-frame requests
  across sessions, in a depot whose two halves look alike, and 1 wrong (mirror) pose in 316
  requests, with vision checks only.
- Tests: the relocalizer in-process (readiness, BUSY, publishing, dry runs, startup, lifecycle,
  parameters), as components, on the fixture against ground truth, and in a launch test where AMCL
  under `nav2_lifecycle_manager` converges after the relocalizer's pose.
- LiDAR verification in the relocalizer (M4). Every pose that passes the vision checks is checked
  against the nearest `LaserScan`, moved to the image's time with odometry: hits, beams that see
  through mapped walls, known space, over a small search window, ramp-aware with an IMU. A place
  counts as a competitor (AMBIGUOUS) only if it passes the check too. `scan_check.mode`: required
  (default), optional or disabled.
- Automatic relocalization (`auto_trigger.*`, off by default): a request when the localization's
  reported uncertainty stays high, then a cooldown.
- Extrinsics monitor (`extrinsics_monitor.*`): while the localization is confident, a dry-run
  request every minute; its successes measure where the camera sits on its mount, and
  `/diagnostics` warns when it moved. Detection only. A service request preempts it.
- Shadow log (`shadow.log_path`): every request that publishes nothing as a CSV row, next to the
  localization's own pose, for field validation.
- `evaluate_scan_check` (landfall_benchmark): the LiDAR check on a recording, at the true poses and
  at wrong ones. `landfall query` checks scans too (`bag.scan_topic` in the build configuration).
- M4 simulation results: 95 to 96 % success and no wrong pose with 3-frame requests across
  sessions; the LiDAR check accepted 99.8 % or more of the true poses and none of 216,100 wrong
  ones.
- Tests: scan check modes, inconsistent odometry, the auto trigger (in-process, and in a launch test
  where it relocalizes a scattered AMCL once), the extrinsics monitor (silent on the recording,
  warns about a camera moved by 3 cm and 2°), preemption, and the scan check on the fixture.
- Learned backends on ONNX Runtime (M5, optional library `landfall_core_onnx`): XFeat local
  features, with the post-processing in C++ following XFeat's `detectAndCompute` and PyTorch's
  `grid_sample` arithmetic, and a generic ONNX global descriptor with EigenPlaces and MegaLoc
  presets. CPU by default; CUDA and TensorRT providers when ONNX Runtime has them (unverified).
- Model management: `landfall_tools/models.yaml` (pinned upstream files, licenses, checksums),
  `scripts/fetch_models.sh`, export scripts for XFeat and EigenPlaces, and MegaLoc's own ONNX file.
  A database records the SHA-256 of its models and refuses a missing or different one.
- `scripts/export_models.sh`: the whole export in one command. It sets up the pinned Python
  packages in a virtual environment, downloads the pinned upstream files with
  `fetch_models.sh --upstream` (each SHA-256 checked) and takes the parity images from the
  fixture bag (`landfall_tools/export/parity_images.py`). Its work files are kept in
  `~/.cache/landfall/export`. `megaloc.py --pytorch` also checks the SHA-256 of upstream's model
  code.
- `landfall build` and `landfall query` take `--models-dir` and `--execution-provider`; the
  relocalizer takes `backend.models_dir` and `backend.execution_provider`.
- `docs/models.md`: presets, usage, the ONNX contracts, parity, and the license and provenance
  audit of the models.
- Tests of the learned path without downloads: synthetic ONNX models and parity files
  (`landfall_core/test/data`), the C++ post-processing against torch's own `grid_sample` (exact),
  and a learned global descriptor end to end through the builder, the database and the localizer.
  A CI job builds without ONNX Runtime; the build also compiles against Ubuntu 26.04's ONNX
  Runtime 1.23.2.
- Nav2 integration (M6): the behavior tree nodes `ReLocalize` (asks the relocalizer for a pose)
  and `IsLocalized` (AMCL's pose is certain enough) in the plugin library `landfall_nav2_bt_nodes`,
  for Nav2 1.1, 1.3 and 1.5 (BehaviorTree.CPP 3 and 4). `ReLocalize` doesn't need its service when
  the tree is loaded, retries a busy relocalizer and waits 20 s for an answer by default.
- Example tree `navigate_to_pose_w_relocalization.xml`, Nav2's default tree of each release with
  relocalization before navigating (when the localization is uncertain) and as the first recovery.
  It relocalizes from up to eight headings (turning 45° after a failed attempt), then clears both
  costmaps, and in the recovery plans the path again from the new pose.
- Launch files in `landfall_bringup`: localization (map_server, AMCL and the relocalizer under one
  lifecycle manager), navigation (nav2_bringup's, with Landfall's tree), both together, and the
  relocalizer alone; `params/relocalizer.yaml` and the per-distro Nav2 overlays
  `params/nav2_<distro>.yaml`. `docs/nav2_integration.md`.
- Simulation: the M6 navigation run (`sim/scripts/navigate_depot.sh`,
  `sim/launch/depot_navigation.launch.xml`): startup relocalization, navigation, a kidnap.

### Changed
- License changed from MIT to Apache-2.0.
- Lyrical is a required CI distro (was allowed to fail).
- The map loader reproduces nav2_map_server's GraphicsMagick conversion exactly (color and
  transparent maps were off by one gray level on Jazzy and later).
- PoseLib is compiled without debug information (build trees shrink by about 2 GB).
- Test timeouts triple in sanitizer builds (`test_map_builder` exceeded 600 s under ASan in CI).
- The recorder's parameter descriptions stand on their own (they appear sorted in the generated
  reference).
- A scan beam violates the map when it runs on for 0.15 m of free space after the last occupied
  cell it crosses; beams that end inside a thick mapped obstacle no longer count.
- Defaults: `ambiguity.max_score_ratio` 0 (any second place that passes every check makes a frame
  AMBIGUOUS), `scan_check.min_hit_ratio` 0.85, `scan_check.refine_pose` false.
- CI reports build errors and failing tests as check annotations, readable without a GitHub login.
- `onnxruntime_vendor` is a dependency of `landfall_core`: the learned backends are built by
  default (`-DLANDFALL_WITH_ONNXRUNTIME=OFF` leaves them out).
- The localizer gives global descriptors the color image (bag of words ignores it).
- CI: `actions/checkout` and `actions/upload-artifact` v7 (Node 24); flake8 checks
  `landfall_tools`.
- CI: runner labels pinned to `ubuntu-24.04`; `ubuntu-latest` is moving to Ubuntu 26.04 (the jobs
  run in ROS containers either way).
- CI: the Jazzy amd64 job fetches the published learned models and parity files (cached until
  `models.yaml` changes) and runs `test_onnx_parity` on them; with `LANDFALL_REQUIRE_MODELS` set
  there, missing models fail the test instead of skipping it.
- `landfall_nav2`'s tests run on isolated ROS domains, `landfall_bringup`'s launch tests in a
  namespace: colcon tests packages in parallel.
- `sim/scripts/start_depot_sim.sh` runs another launch with `SIM_LAUNCH`.
- The M0 compile probe `test_compat_bt` gave way to the behavior tree nodes' tests.

### Fixed
- The auto-trigger node test expected no third automatic request after shortening the cooldown,
  but with the localization still uncertain the trigger rightly keeps firing; it failed on GitHub's
  arm64 runner. The odometry-consistency and preemption tests no longer depend on timing, and the
  extrinsics monitor tests take one-frame samples.
- The model exports would have failed: `xfeat.py` couldn't import its helper module when run with
  `python3 -I`, the documented invocation, and neither network exported with a dynamic image size.
  XFeat's 8x8 unfolding (`Tensor.unfold`) is swapped for `pixel_unshuffle`, which gives the same
  values bit for bit (the script checks). EigenPlaces' GeM pooling takes a mean instead of a
  pooling kernel the size of the feature map. Found by exporting with random weights.
- The accuracy table in `docs/relocalization.md` counted images instead of requests: a frame that
  finds nothing doesn't end a request, so run 1 gives 434 successes in 440 requests, not 434 of
  448 (re-run in M5; same successes and wrong poses).
