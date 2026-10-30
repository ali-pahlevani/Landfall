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

### Changed
- License changed from MIT to Apache-2.0.
- Lyrical is a required CI distro (was allowed to fail).
