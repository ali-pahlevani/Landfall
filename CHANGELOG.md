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
