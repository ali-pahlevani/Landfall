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

### Changed
- License changed from MIT to Apache-2.0.
- Lyrical is a required CI distro (was allowed to fail).
- The map loader reproduces nav2_map_server's GraphicsMagick conversion exactly (color and
  transparent maps were off by one gray level on Jazzy and later).
- PoseLib is compiled without debug information (build trees shrink by about 2 GB).
