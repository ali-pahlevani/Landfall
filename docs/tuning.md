# Tuning

Landfall's defaults err toward misses: a request that publishes nothing is safe, a wrong pose is
not. Change a threshold only with numbers from your own site, and check what it does to wrong
poses before what it does to successes. All parameters are listed in `docs/parameters.md`.

## How to measure

- **Offline, on a recording.** `landfall query` replays a bag's images through the relocalizer's
  pipeline and decision, with `--set name=value` to try another threshold (the node parameter's
  name); with ground truth, `evaluate_poses --queries` scores the results
  (`docs/relocalization.md`). Without ground truth, compare with a well-localized AMCL in the bag.
- **On the robot, without publishing.** The shadow log (`shadow.log_path`) records every
  non-publishing request next to the localization's own pose: dry runs and the background checks
  (`docs/field_validation.md`).
- **In simulation.** The kidnap benchmark (`docs/benchmark.md`).

## Recording

- **Localization uncertainty gate** (`gating.max_std_xy`, `gating.max_std_yaw_deg`, 5 cm and 2°).
  AMCL's covariance is its own estimate, not its error: in the depot simulation it reported about
  five times its actual error (PLAN V16), so the default rejects many good poses there (the
  simulation's build uses 8 cm / 3°). It can also be confident and wrong in a corridor. Look at the
  recorder's rejection counts (`~/status`) and the build report's coverage map.
- **Keyframe poses** (`poses.*` of the build, `docs/recording.md`). Record the scan with the
  keyframes (`scan.topic`): the build then matches every keyframe to the map instead of trusting
  the localization's pose. The build report's "Keyframe poses" section tells how often the scan
  contradicted the recorded pose; many contradictions mean the localization drifted during the
  recording. `poses.recorded_std_*` is how far a recorded pose that agrees with its scan is
  trusted in the directions the scan confirms: lower it only if your localization is better than
  the map's cells (5 cm). Few contradictions don't prove good poses: where one scan fits two
  positions equally (a wall the map has twice, repeating shelf legs), the localization and the
  scan can agree on the wrong one (`docs/recording.md`, "Limit").
- **Speed gate** (`gating.max_linear_speed`, `gating.max_angular_speed`). It bounds the error that
  image and odometry timing leave in the keyframe poses; with a measured `image.time_offset` it can
  be relaxed a little.
- **Coverage.** Drive every lane in both directions: the relocalizer finds only views the
  recording saw (M6: a robot that stopped in an unrecorded lane couldn't relocalize). The build
  report's heading rose per cell shows the gaps.

## Requests

- **Frames per request** (`consistency.num_frames`, 3). Keep 3. In the depot, single-frame requests
  published the mirror pose of the hall's other half once per direction; the 3-frame requests
  holding those frames were rejected by the odometry check (`docs/relocalization.md`). Fewer frames
  are faster; more frames catch more and miss more.
- **Timeout** (`query.timeout`, 10 s, or the request's own). A request ends after its frames or at
  the timeout. The example behavior tree asks for 2.5 s per heading and turns between attempts,
  which recovers more than one long request at a heading the database has no views from.
- **Consistency** (`consistency.max_translation_error`, `consistency.max_rotation_error_deg`,
  0.15 m and 5°). Raise the rotation limit if requests end `REJECTED_CONSISTENCY` while the robot
  turns: wheel odometry slips on the spot (`docs/sensors.md`).

## Checks

- **Scan check** (`scan_check.*`). `scan_check.min_hit_ratio` 0.85 comes from the depot drives: at
  every scan's true pose and at 213,240 wrong poses 0.5 to 5 m away, 0.60 accepted 344 wrong
  poses, 0.80 accepted 9 and 0.85 none, while the true poses accepted barely changed (10,652 of
  10,662). `evaluate_scan_check` runs the same test on your recordings. `scan_check.hit_tolerance`
  must be at least one map cell. Objects that the map doesn't have make short readings (misses),
  which the hit ratio tolerates up to 15 %; objects the map has and the scene lost make beams
  violate the map, and the check refuses. Both cost successes, not wrong poses: in the kidnap
  benchmark, nine stacks of goods off the map halved the successes (93 to 46 %), almost all through
  this check, as a stack within about 3 m blocks enough beams, and removed crate stacks cut them to
  68 % (`docs/benchmark.md`). Where goods stand off the map, measure the true poses' hit ratios on
  your own recordings before lowering the limit: in the depot, 0.80 already let 9 wrong poses
  through.
- **Scan alternatives** (`scan_check.check_alternatives`, `scan_check.alternative_margin`, 0.01).
  After a pose passes, the scan is fitted everywhere else; a place it fits better by the margin
  makes the frame `AMBIGUOUS`. In the depot no other place fitted better than the true pose at any
  of 1,927 scans (the best was at least 0.015 worse), while near the building's mirror poses the
  true pose always fitted better (by 0.006 at least). A margin of 0 would also make places that fit
  alike to within noise ambiguous (twin aisles, which vision should decide); a larger one lets more
  look-alikes through. `evaluate_scan_check --alternatives` (and `--symmetry` for a building that
  looks alike turned about a point) measures both on your recordings. Turning the check off saves
  about 20 ms per frame and gives up the defense against look-alike places that the LiDAR check
  alone lets through.
- **Ambiguity** (`ambiguity.max_score_ratio`, 0). Any second place that passes every check, the
  LiDAR included, makes the frame `AMBIGUOUS`, whatever its score: in the depot's mirrored halves
  the wrong place out-scored the right one in 8 of 20 ambiguous frames. Leave it at 0.
- **Sanity** (`sanity.*`). The height and tilt limits assume the extrinsics are right: a camera
  that moved on its mount makes requests fail here (the extrinsics monitor reports it). The inlier
  limits (`sanity.min_inliers`, 40) apply to every backend.

## Triggers and monitors

- **Auto trigger** (`auto_trigger.*`): fires when the localization reports itself uncertain for
  `duration` seconds. A localization that is confidently wrong doesn't: AMCL after the M6 kidnaps,
  and after global localization in the benchmark, reported a few centimetres while metres off.
  That case is the kidnap watchdog's.
- **Kidnap watchdog** (`kidnap_watchdog.*`): a shorter `period` notices a kidnap sooner and costs one
  request per period. The limits (1 m, 20°) are far above AMCL's normal error; keep
  `confirmations` at 2 or more.
- **Extrinsics monitor** (`extrinsics_monitor.*`): `max_amcl_std_xy` (5 cm) must be at or above
  what your AMCL reports while it is right, or the monitor never samples.

## Backends

`classical` (ORB, bag of words) is the default and the faster one: about 50 ms per frame in the
depot on a laptop CPU, LiDAR checks included. `learned_cpu` (XFeat, EigenPlaces) takes several
times longer on the CPU and, in the depot, succeeded less often than `classical`
(`docs/models.md`, `docs/benchmark.md`).
Compare both on your recordings before choosing.
