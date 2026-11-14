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
  By default it applies only to frames without a scan (`gating.covariance: without_scan`): record
  the scan (`scan.topic`) and the build checks each keyframe against the map instead.
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
  scan can agree on the wrong one; the report counts those keyframes ("Scans that fit a second
  place") and draws them orange on the coverage map (`docs/recording.md`, "Limit").
  `poses.disagreement_std_xy` (1 m) is how much the recorded pose counts where the scan can't
  confirm it; lower it only for a localization with information the scans lack (e.g. another
  sensor), since one built on the same scans is no better there.
- **Odometry noise** (`poses.odom_std_*` of the build). `poses.odom_std_xy_per_rad` (0.1 m per
  radian turned) is how much a turn may throw the odometry's position off: about 0.015 m for the
  depot simulation's TurtleBot 4, 0.2 m for a skid-steer robot whose wheels slip. The report's
  "Odometry against the scans" row measures it on your recording, and a warning names the value
  when it is more than a quarter above the setting (set it to that value); too low a setting lets
  a slipping turn drag the keyframes after it, too high one costs little where the scans pin the
  poses.
- **Floor mask** (`features.floor_mask_distance` of the build, 5 m; `docs/recording.md`). The
  floor nearer than this is hidden from the feature detector, in the build and in queries.
  In the URDF Maker warehouse 3 m and 5 m did equally well (relocalization success 9.5 and 9.6 % of
  the sweep frames, 6.4 % without the mask), 7.6 m and 12 m less (9.0 and 8.1 %; farther masks
  also hide the bottoms of nearby shelves). Lower it, or set 0, if the build report says the mask
  hides most of the image (a camera looking down at the floor).
- **Speed gate** (`gating.max_linear_speed`, `gating.max_angular_speed`). It bounds the error that
  image and odometry timing leave in the keyframe poses; with a measured `image.time_offset` it can
  be relaxed a little.
- **Coverage.** The relocalizer finds only views the recording saw (M6: a robot that stopped in an
  unrecorded lane couldn't relocalize). Drive every lane both ways, open floor in lanes at most 2 m
  apart, and turn on the spot at aisle ends and junctions (`docs/recording.md`, "Where to drive").
  The build report shades the floor the robot can reach by the directions it has views of, and the
  build warns below `coverage.target` (80 % of that floor with a view within 1 m).

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
  benchmark, nine stacks of goods off the map halved the successes (93 to 46 %, 52 % since PLAN
  V61), almost all through this check, as a stack within about 3 m blocks enough beams, and removed
  crate stacks cut them to 68 % (76 %; `docs/benchmark.md`). Where goods stand off the map,
  measure the true poses' hit ratios on your own recordings before lowering the limit: in the
  depot, 0.80 already let 9 wrong poses through.
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
- **What the scan tells apart** (`sanity.min_inliers_unique`, 20; `scan_check.unique_margin`,
  0.05; `scan_check.max_twin_support`, 0.5; `docs/relocalization.md`). A pose with 20 to 39
  inliers stands only where every other place fits the scan at least the margin worse and no place
  within 1 m fits almost as well. In the URDF Maker warehouse that took single requests from 22.8 to
  27.3 % with no wrong pose; 20 inliers without the condition gave 30.1 % and 4 wrong poses, one
  6.6 m off along a repeating wall. Set `sanity.min_inliers_unique` to 40 to need 40 inliers
  everywhere. The weakest right poses there had gaps of 0.05 and more; a smaller margin also accepts
  places the LiDAR barely tells apart. `scan_check.max_twin_support` refuses poses that rest on
  keyframes the build flagged (`fits a second place` warning): there the poses with at most half
  of their inliers from them were at most 22 cm off (median 3.5 cm), those with 90 % or more up to
  0.56 m (median 26 cm). Raising it gains poses in such aisles and risks half-metre errors; fixing
  the map's doubled walls gains them safely.
- **Sanity** (`sanity.*`). The height and tilt limits assume the extrinsics are right: a camera
  that moved on its mount makes requests fail here (the extrinsics monitor reports it). The inlier
  limits (`sanity.min_inliers`, 40) apply to every backend. `sanity.min_inlier_ratio` is 0.1:
  repeated structure (boxes, shelving) adds many wrong 2D–3D matches around a right pose; in the
  warehouse 0.25 refused 28 right poses that 0.1 kept, and no wrong one passed.

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
