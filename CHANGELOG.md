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
- Kidnap watchdog in the relocalizer (M7, off by default, `kidnap_watchdog.*`): dry-run checks of
  the localization's pose, every 10 s; after two in a row that pass every check but disagree with
  the localization by more than 1 m or 20°, a relocalization (trigger `watchdog`). The extrinsics
  monitor's checks count too.
- Scan alternatives (M7): once a pose has passed every check, the relocalizer fits the LiDAR scan
  everywhere else in the map; a place it fits better by `scan_check.alternative_margin` (0.01)
  makes the frame AMBIGUOUS (`scan_check.check_alternatives`, on). It stops a building's look-alike
  place that vision found and the LiDAR check alone let through: the benchmark's first development
  run published two such wrong poses in the depot.
- `summarize_shadow` (landfall_benchmark): the relocalizer's shadow log against the localization,
  per trigger, with the 95 % upper bound of the disagreement rate (field validation).
- `landfall_tools/install_onnxruntime.sh`: official ONNX Runtime 1.24.3 builds (CPU, GPU with
  CUDA 12, aarch64), checked against their SHA-256, for `-DONNXRUNTIME_ROOT`.
- `landfall_bringup/rviz/landfall.rviz`: map, scan, AMCL's particles and pose, the robot, the
  relocalizer's markers and debug image.
- Documentation: `docs/architecture.md`, `docs/sensors.md` (camera, odometry fusion, IMU, LiDAR,
  frames), `docs/tuning.md`, `docs/troubleshooting.md`, `docs/field_validation.md` (the
  procedure on a real robot), `docs/benchmark.md`.
- Humble simulation: the depot on the TurtleBot 4's own simulator (Gazebo Fortress):
  `sim/scripts/start_depot_sim_humble.sh`, `record_depot_humble.sh`, `navigate_depot_humble.sh`,
  `sim/launch/depot_humble.launch.xml`, `sim/params/localization_depot_humble.yaml`. The M7 run:
  the recording, a database, the startup relocalization 4.3 cm from the truth and the first goal;
  after the kidnap the robot stopped where the recording never drove, and the goal aborted
  (`docs/nav2_integration.md`).
- `evaluate_poses` reads the ground truth as `nav_msgs/Odometry` or `tf2_msgs/TFMessage` too
  (`--gt-child`), as the TurtleBot 4 simulator publishes it.
- `evaluate_scan_check --symmetry`, `--symmetry-search`, `--sweep` and `--alternatives`: the LiDAR
  check at a building's look-alike poses, other thresholds, and how much better the scan fits
  elsewhere. `sample_placements` prints the benchmark's placements of a seed.
- The kidnap benchmark (M7, `docs/benchmark.md`): `kidnap_runner` moves the simulated robot to
  random places and measures how Landfall (looking around from up to eight headings), AMCL's
  global localization or the kidnap watchdog give it its pose back; `summarize_benchmark` writes
  the results tables with 95 % Clopper-Pearson bounds. Scenarios: `landfall_benchmark/scenarios/`;
  depot world variants (other light, added goods, crates removed): `landfall_bringup/sim/worlds/`;
  `sim/scripts/benchmark_depot.sh` runs it; `landfall_tools/plot_benchmark.py` draws the figures.
- M7 benchmark results (seed 2027, 100 placements per row; `docs/benchmark.md`, README): no wrong
  pose from Landfall in 800 placements. Success with `classical` / `learned_cpu`: 93 / 78 % in the
  depot as mapped, 91 / 77 % in other light, 46 / 44 % with nine stacks of goods the map doesn't
  have (the LiDAR check refuses near them), 68 % (`classical`) with crate stacks gone that the map
  still has. AMCL's global localization: 6 to 8 % success and 82 or 83 wrong poses per 100. The
  kidnap watchdog undid 85 % of the kidnaps (median 39 s) without a needless relocalization. Result
  files in `landfall_benchmark/results/seed2027/`, figures in `docs/images/`.
- `CONTRIBUTING.md`, and `.pre-commit-config.yaml` with CI's formatting, flake8 and policy checks
  at the same versions.
- Docker workflow: the runtime image of each distro, built and started on every push to main and
  on version tags; pushed to the GitHub Container Registry when the repository variable
  `LANDFALL_PUBLISH_IMAGES` is `true`.
- Keyframe poses from scan matching and odometry (PLAN V56): the recorder stores each keyframe's
  LiDAR scan (`scan.topic`, `scan.max_time_offset`) and odometry pose; `landfall build` matches
  every scan to the map near the recorded pose, fuses the match with the recorded pose (fully only
  where they agree, and only in the directions the scan confirms), ties consecutive keyframes by
  odometry in a pose graph, and starts pair selection, triangulation and the bundle adjustment
  from it (builder keys `poses.*`, `bag.scan_max_time_offset`). The build report gains a
  "Keyframe poses" section, keyframes the flags `kFlagScanMatched` and `kFlagScanUnmatched`.
  `landfall_core`: `matchScan` (correlative scan-to-map matching; its covariance counts the
  surfaces' directions and any near-equal second fit in the search window), odometry residuals in
  the bundle adjustment. On the URDF Maker warehouse drive (372 keyframes, AMCL recording) the
  keyframe poses' error fell from 12.1 / 44.3 cm (median / 90th percentile) to 3.4 / 32.6 cm, and
  the database's landmarks from 4567 to 5376 (5378 with true poses). The rest of the tail is
  aisles where one scan fits two positions equally and AMCL sat on the wrong one (T-011).

### Changed
- The recorder and `landfall build --bag` no longer gate frames that come with a LiDAR scan on the
  localization's covariance (PLAN V57; `gating.covariance`, `without_scan` by default, `always`
  for the old behaviour): the build matches those frames to the map. A keyframe whose scan doesn't
  match the map now counts its recorded pose only loosely (`poses.disagreement_std_*`), so its
  odometry carries it, and where the scan agrees the recorded pose counts less the less precise
  the scan is in a direction (std scan std² / `poses.recorded_std_xy`). On the URDF Maker warehouse
  route: 594 keyframes instead of 372, 8,698 landmarks instead of 5,376, relocalization success
  4.1 % -> 6.4 % of the 904 sweep frames (6.1 % with true poses), no wrong pose (live settings).
- The feature detector no longer sees the floor near the robot (PLAN V58;
  `features.floor_mask_distance`, 5 m unless set, 0 keeps the floor): the build hides the pixels
  whose ray meets the floor nearer than that, from each keyframe's camera model and base -> camera
  TF, and stores the distance with the local features, so queries hide the same floor. On the URDF
  Maker warehouse route (fix 2's recording): 15,039 landmarks instead of 8,701, relocalization
  success 6.4 % -> 9.6 % of the 904 sweep frames; with true keyframe poses 6.1 % -> 9.2 %, no
  wrong pose. With AMCL's poses, 3 of the new successes are 0.51-0.52 m off, in the southern
  aisles where the nearby keyframes are 0.45-0.55 m off (T-011). Databases built before keep
  working without a mask; rebuild them to use it. The build report and `landfall build` give the
  features per keyframe and how much of the image the mask hid.
- `ILocalFeatures::extract` takes the image's camera and its pose on the robot (`ImageGeometry`).
- Keyframe poses where the scan can't tell (PLAN V59, T-011): odometry links count less the more
  the robot turned (`poses.odom_std_xy_per_rad`, 0.1 m per radian), and the recorded pose counts
  only at `poses.disagreement_std_xy`, now 1 m, where the scan contradicts it or can't tell a
  direction. The build report checks the recording against these assumptions: keyframes whose scan
  fits a second place almost as well (a wall the map has twice; flagged, orange on the coverage
  map, a warning above 5 %), and the odometry against the scan matches (a warning when turns err
  more than the setting); it also warns when the floor mask hides more than 60 % of the image. On the URDF Maker warehouse route: wrong poses 3 -> 0, median error
  7.3 -> 5.2 cm, success 9.6 -> 9.3 % of the sweep frames; in the depot simulation unchanged
  (98.2-98.6 %, no wrong pose).
- CI: the Docker images retry their network steps (apt update and downloads, rosdep update and
  install).
- Staging folders are format 2: `keyframes.csv` gains the odometry pose, `scans.csv` the scans.
  Format 1 folders still build.
- The bundle adjustment's bounds around a scan-matched keyframe widen to 3 standard deviations of
  its prior (at least `ba.max_shift_xy` / `ba.max_shift_yaw_deg`); its shift statistics are
  measured from the pose graph's poses.
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
- The relocalizer's accuracy table (`docs/relocalization.md`) is re-run with the scan alternatives
  check: the same successes, no wrong single-frame pose.
- The simulation image builds for Humble too (the TurtleBot 4 simulator on Gazebo Fortress, with
  Cyclone DDS); `scripts/sim.sh` takes `LANDFALL_SIM_DISTRO`.
- The M0 compile probe `test_compat_bt` gave way to the behavior tree nodes' tests.
- README: the benchmark results, how it works, a quick start and the documentation index.

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
