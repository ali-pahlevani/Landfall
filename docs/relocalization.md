# Relocalization

`landfall_relocalizer` finds the robot on the map from camera images and hands the pose to AMCL.
It localizes each image against a map database built with `landfall build`
([recording.md](recording.md)). Then it checks the result three ways: is it plausible, does the
LiDAR scan fit the occupancy map there, and do several frames agree with odometry? Only then does
it publish `/initialpose`, or call AMCL's `set_initial_pose` service. Vision proposes, LiDAR and
odometry verify. A request that can't be sure answers with a reason and publishes nothing.

## Running it

```bash
ros2 run landfall_ros relocalizer_node --ros-args \
  -p database_path:=/path/to/site.landfall.db \
  -p image.topic:=/camera/image_raw -p image.transport:=raw \
  -p camera_info.topic:=/camera/camera_info \
  -p scan_check.topic:=/scan \
  -p frames.base:=base_footprint \
  -p autostart:=true
```

The node is also a component, `landfall::ros::RelocalizerNode`. It is a lifecycle node with a bond,
so it can be listed after `amcl` in the `node_names` of `lifecycle_manager_localization`.
`landfall_bringup`'s launch files do that, and Nav2's navigator can ask it for a pose through the
behavior tree node `ReLocalize`: see [`docs/nav2_integration.md`](nav2_integration.md).

A missing or unusable database doesn't fail the *configure* transition, because it must not stop
AMCL's lifecycle manager. Instead, every request answers `NOT_READY` with the reason, and
`/diagnostics` reports an error.

Ask for a pose:

```bash
ros2 service call /landfall_relocalizer/relocalize landfall_msgs/srv/Relocalize \
  "{num_frames: 0, timeout: 0.0, dry_run: true}"   # 0: use the node's parameters
```

The answer, also latched on `~/status`, is a `landfall_msgs/RelocalizationResult`. It carries a
status, a message, the pose when the status is `SUCCESS`, the worst scores across the frames used,
and the hypotheses of the last frame. A request that arrives while another one runs is answered
`BUSY` at once.

## What a request checks

Before it uses any image, the request answers **`NOT_READY`** when one of these is missing:

| Reason in the message | Fix |
|---|---|
| `map database not loaded: …` | `database_path`, or rebuild the database (schema, backend versions) |
| `no occupancy grid on 'map' yet` / `the map on 'map' differs from the database's (…)` | Run map_server with the map the database was built from; `require_map_match: false` skips the check |
| `… is not the map YAML/image the database was built with` | `map_yaml` is set and points at other files |
| `no camera_info on '…'` / `the camera is W×H, the database was built for W×H` | Camera topics, or rebuild for this camera |
| `nothing subscribes to /initialpose` / `AMCL (amcl) is inactive, not active` | Only when publishing: AMCL must be up and active (`output.amcl_node`; empty skips the check) |
| `no LaserScan on 'scan' (scan_check.mode is required)` | `scan_check.topic`; no LiDAR at all: `scan_check.mode: optional` or `disabled` (see *The LiDAR check*) |
| `no transform odom -> base_footprint at the image` / `… -> camera` | TF at the image stamps (see `tf_timeout` and `image.time_offset`) |
| `no scan on 'scan' within 0.50 s of the image` | Scan and image clocks (`scan_check.max_time_offset`) |
| `no image on '…' within N s` | Image topic and transport |

Then it collects frames at least `consistency.min_frame_interval` apart. It continues until
`consistency.num_frames` (default 3) of them produced a pose, or `query.timeout` runs out. Each
frame goes through these steps:

1. Extract features. Retrieve the `query.top_k` most similar keyframes and group them into places,
   by shared landmarks or by camera proximity.
2. Match each place 2D–3D, then solve the pose: upright LO-RANSAC with the gravity from the IMU or
   the flat floor, falling back to P3P.
3. Run the sanity checks (`sanity.*`): inliers, inlier ratio, reprojection error, image coverage,
   base height and tilt, and a free map cell under the base. A pose needs `sanity.min_inliers` (40)
   inliers, or `sanity.min_inliers_unique` (20) where the LiDAR scan tells the place apart (step
   6), and `sanity.min_inlier_ratio` (0.1) of its 2D–3D matches as inliers.
4. Check the LiDAR scan nearest the image against the map at the pose (`scan_check.*`, below).
5. Cluster the poses that passed every check. A second, separate place that passed them too, with
   at least `sanity.min_inliers` inliers, makes the frame ambiguous (with fewer than 40 inliers
   for the best place, any second place does). The default `ambiguity.max_score_ratio: 0` counts it
   whatever its score: in a building whose halves look alike, the wrong half can score higher than
   the right one.
6. Fit the scan elsewhere (*Where else the scan fits* below): a place it fits better makes the frame
   ambiguous, and so does a pose whose landmarks came mostly from keyframes the build couldn't
   place; a pose with fewer than `sanity.min_inliers` inliers stands only where the scan fits
   nowhere else nearly as well.

The decision:
- Any ambiguous frame makes the request **`AMBIGUOUS`**.
- Too few valid frames give the deepest failure reached: **`NO_MATCH`** (no pose with enough
  inliers), then **`REJECTED_SANITY`**, then **`REJECTED_SCAN_CHECK`** (also for a pose with fewer
  than `sanity.min_inliers` inliers that the scan doesn't tell apart).
- Frames that disagree with the odometry between them by more than
  `consistency.max_translation_error` or `consistency.max_rotation_error_deg` give
  **`REJECTED_CONSISTENCY`**.
- Otherwise the request succeeds. The pose is moved to the latest odometry
  (`output.compensate_motion`) and published with σ = `output.std_xy` and `output.std_yaw_deg`.

Every threshold except the structural parameters can change at runtime with `ros2 param set` and
applies to the next request ([parameters.md](parameters.md)).

## The LiDAR check

For every pose that passed the sanity checks, the node takes the `LaserScan` nearest the image
(at most `scan_check.max_time_offset` apart). It moves the scan to the image's time with odometry
and lays its beams on the occupancy map at the pose. Beams outside `range_min` and
`scan_check.max_range` don't count; neither do beams that end in unknown space.
- A **hit** is a beam that ends within `scan_check.hit_tolerance` of an occupied cell.
- A **violation** is a beam that runs on for at least 0.15 m of free space after the last occupied
  cell it crosses: the scan sees through a wall that the map has.
- The pose passes if, somewhere in a small window around it (`scan_check.search_radius`,
  `scan_check.search_angle_deg`), at least `scan_check.min_hit_ratio` (0.85) of the beams are hits
  and at most `scan_check.max_violation_ratio` (0.05) are violations. At least
  `scan_check.min_valid_beams` beams must count, and at least `scan_check.min_known_ratio` of them
  must end in known space.

People and pallets that weren't mapped make beams *short*. Those are misses, never violations, and
up to 15 % of them are tolerated. A wrong pose makes beams end in the middle of mapped free space or
pass through mapped walls. By default the published pose stays vision's; `scan_check.refine_pose`
publishes the best pose of the window instead. In the depot simulation, that was less accurate than
vision: a median error of 2.1 and 1.9 cm in the two directions, against 1.3 and 1.1 cm.

With an IMU on a ramp, beams are projected with the IMU's tilt, and beams that would hit the floor
are dropped.

**Where else the scan fits.** A building can look alike in two places: the depot's two halves do,
turned 180° about its centre. When the recording saw the wrong one of them from where the robot
looks and the right one only from elsewhere, vision finds the wrong place, the right one never
competes, and the LiDAR check can pass: near the depot's mirror poses, the scan fits well enough
for a fifth of the scans. So once a pose has passed every check (`scan_check.check_alternatives`,
on), the node fits the scan everywhere else in the map, at every heading. If it fits another place,
beyond 1 m or 30°, at least `scan_check.alternative_margin` (0.01) better than the pose (in hit
ratio less violation ratio), the frame is `AMBIGUOUS`: the message says the scan fits another place
better. Places that are truly identical for the LiDAR, such as twin aisles, fit alike and stay
vision's to decide. The search takes about 20 ms per frame on 16 threads.

**What the scan tells apart.** Where the LiDAR settles the place by itself, fewer inliers suffice. A
pose with at least `sanity.min_inliers_unique` (20) but fewer than `sanity.min_inliers` (40)
inliers passes only where the scan fits every other place (beyond 1 m or 30°) at least
`scan_check.unique_margin` (0.05) worse, and no place within 1 m almost as well: in the window the
build matches keyframe scans in (±1 m, ±6°), the scan must fit best at the pose, and a second
fit, at least 15 cm away, must be at least 1 nat worse. Otherwise the frame is refused, and its best
hypothesis says why (`… inliers < 40 where the scan fits a place 0.47 m away almost as well`).
Without the search (`scan_check.check_alternatives: false`) or without a scan, 40 inliers it is.

**Landmarks the build couldn't place.** The build flags the keyframes whose scan fits a second place
within 1 m almost as well, typically a wall the map has twice (`docs/recording.md`, *Keyframe
poses*). Along it their poses rest on odometry and the recorded pose, so they may sit at the other
place, and so may the landmarks they triangulated and a pose found from them. A pose whose inliers
come for more than `scan_check.max_twin_support` (0.5) from such keyframes is `AMBIGUOUS` (`the
landmarks come from keyframes whose LiDAR scans fitted two places within 1 m`). The scan can't
settle it there: where the map is wrong, it may fit the wrong place better. In the URDF Maker
warehouse's aisles with doubled walls, a pose 0.56 m off fitted the map 1 nat better than the true
one. The more of a pose's inliers came from flagged keyframes, the further off it was: with at most
half of them, a median 3.5 cm and at most 22 cm (256 poses); with 90 % or more, a median 26 cm and
up to 0.56 m (19 poses). Maps without flagged keyframes, such as the depot's, never meet this
check.

In the warehouse (the 904 sweep requests against the protocol drive's database,
`docs/recording.md`), these settings localized 27.3 % of the single requests instead of 22.8 %, with
no wrong pose instead of 2, at a median error of 3.1 cm instead of 3.4. A look-around (eight
headings) localized 75 of the 113 places instead of 85: 61 of the 66 in the open north instead of
59, and 14 of the 47 in the aisles with doubled walls instead of 26, where most poses rested on
flagged keyframes (median error 26 cm where at least 90 % of the inliers came from them). With 20
inliers and no condition, 30.1 % localized and 4 were wrong (one 6.6 m off along a repeating wall).
In the depot simulation's cross-session requests (*Accuracy* below), the settings changed nothing:
98.7 % and 98.2 % success, no wrong pose. In the kidnap benchmark they took the depot with unmapped
goods from 46 to 52 % and the one with removed crates from 68 to 76 %, still without a wrong pose
(`docs/benchmark.md`).

`scan_check.mode` is `required` by default: no request runs without a scan. `optional` checks a
pose when a scan is there and accepts it on the vision checks otherwise. `disabled` never checks.
Use either only on a robot without a LiDAR the map matches: the LiDAR check is what stops a place
that merely looks right.

The check works on its own data too. `evaluate_scan_check` runs it on a recording with ground truth:
at every scan's true pose, where it must pass, and at wrong poses 0.5 to 5 m away, where it must
fail (*Accuracy* below). For a building that looks alike turned about a point, `--symmetry CX CY
ANGLE_DEG` also checks every scan at its true pose turned that way (`--symmetry-search` widens the
window, as vision's pose can be anywhere near it), `--sweep MIN_HIT MAX_VIOLATION` counts what other
thresholds would accept, and `--alternatives` measures how much better the scan fits elsewhere, at
the true poses and at the symmetric poses that pass.

## Automatic relocalization

With `auto_trigger.enabled: true`, the node watches the localization's own uncertainty on
`auto_trigger.pose_topic` (AMCL's `amcl_pose`). If the reported standard deviation stays above
`auto_trigger.max_std_xy` or `auto_trigger.max_std_yaw_deg` for `auto_trigger.duration` seconds,
it runs a request on its own (trigger `auto`). After that it waits at least `auto_trigger.cooldown`
seconds before the next one. It never starts while another request runs.

AMCL publishes `amcl_pose` only after a filter update, so a robot that stands still keeps its last
estimate. An AMCL that never received a pose publishes nothing, so this trigger stays quiet: that
case is what startup relocalization is for.

## Kidnap watchdog

After a kidnap, AMCL can stay confident on a wrong pose. Nav2 notices only when the robot can't
move, so until then the robot drives on blind (docs/nav2_integration.md). With
`kidnap_watchdog.enabled: true`, the relocalizer checks the localization itself: every
`kidnap_watchdog.period` seconds (default 10), while the localization has a pose, it runs a full
request without publishing (trigger `watchdog`). Like the extrinsics monitor's requests, these
checks give way to any other request and never appear on `~/status`.

A check that passes every test is compared with the localization's pose at the check's stamp. It
disagrees when the two are further apart than `kidnap_watchdog.max_translation` (1 m) or
`kidnap_watchdog.max_rotation_deg` (20°). The extrinsics monitor's checks count too; checks that
find no pose say nothing. After `kidnap_watchdog.confirmations` (2) disagreeing checks in a row,
with no agreeing one between them, the node runs a normal relocalization (trigger `watchdog`),
which publishes only if every check passes again, and no other within `kidnap_watchdog.cooldown`
seconds (30). Any published pose restarts the count, which `/diagnostics` shows.

The cost is one request every 10 s. The risk is a relocalization that a right localization didn't
need; the kidnap benchmark measures how fast the watchdog notices a kidnap and how often it
relocalizes a robot whose localization was right (docs/benchmark.md).

## Startup relocalization

With `startup.enabled: true`, the node relocalizes on its own after activation. It starts after
`startup.delay` and retries every `startup.retry_interval` until one attempt succeeds. Attempts
answered `NOT_READY` don't count; after `startup.max_attempts` attempts that ran, it gives up. Pair
it with AMCL's `always_reset_initial_pose: true` (PLAN §7.16).

## Extrinsics monitor and shadow log

A camera that turned on its mount (yaw) or slid sideways still passes the height and tilt checks;
localization just gets worse and worse. The extrinsics monitor (`extrinsics_monitor.enabled`, on by
default) watches for that. It only reports and never changes a transform: recalibration stays a
URDF change.

Every `extrinsics_monitor.period` seconds (default 60), while the localization reports a standard
deviation below `extrinsics_monitor.max_amcl_std_xy`, the node runs a full request without
publishing (trigger `monitor`). A service request preempts it instead of being answered `BUSY`.
When it succeeds, its latest frame where the robot was slow (`extrinsics_monitor.max_linear_speed`,
`extrinsics_monitor.max_angular_speed`) gives one sample: the camera pose from vision against the
one AMCL and TF predict. Once `extrinsics_monitor.min_samples` samples are in, the per-axis median
is the camera's offset on its mount. `/diagnostics` warns above `extrinsics_monitor.warn_translation`
or `extrinsics_monitor.warn_rotation_deg` and reports an error at 2.5 times those.

A robot that drives without stopping gives few samples: in the depot drives, only about 4 % of the
images were taken while the robot was confident and slow.

With `shadow.log_path` set, every request that publishes nothing appends a CSV row. That covers dry
runs, the monitor's requests and the watchdog's checks. Each row holds the status, the pose, and the localization's own
pose and standard deviation at the same stamp. Columns:

```
stamp_ns,trigger,status,frames_used,pose_stamp_ns,x,y,yaw,localization_x,localization_y,
localization_yaw,error_m,error_deg,localization_std_xy,localization_std_yaw_deg,inliers,
scan_hit_ratio,elapsed_s,message
```

This is shadow mode. Leave the node running beside AMCL, with the monitor's period shortened, and
the log shows how often relocalization would have succeeded, missed or been wrong, without ever
publishing. The field validation (PLAN M8) is built on it.

## Diagnostics and debugging

- `/diagnostics`: readiness, database size and hash, the startup extrinsics check, the extrinsics
  monitor, the kidnap watchdog's count, and the last result.
- `~/markers`: every solved hypothesis (green valid, orange rejected), the keyframes of the winning
  place, and the published pose (blue).
- `~/debug_image`: the last frame with the winning hypothesis' inliers (green) and outliers (red).
  It is rendered only while something subscribes.
- `debug.dump_dir`: every query frame as a JPEG, plus one JSON line per frame in `queries.jsonl`
  (stage, inliers and rejection reason of every hypothesis). Use it for offline tuning.
- The log has one line per request with its status, duration, frames and milliseconds per frame.

**Startup extrinsics check.** When `base → camera` is made of `/tf_static` transforms only, the node
compares it with the transform stored in the database. It warns on `/diagnostics` above
`extrinsics_check.warn_translation` or `extrinsics_check.warn_rotation_deg`, which usually means the
database was built for another camera mount or URDF. A camera on a moving joint, such as a fork
carriage, skips this check. The extrinsics monitor covers that case.

**Cameras that stamp images when they publish them.** Some drivers stamp an image when they publish
it, not at exposure, so poses are looked up a few tens of milliseconds late. Set
`image.time_offset` to the negative latency, the same value as for recording.

## Offline evaluation

`landfall query` replays a bag's images through the same localizer and decision. Its options:
- `--frames N` emulates requests of N frames.
- `--every` thins the images.
- `--skip-keyframes` leaves out the database's own images.
- `--set` changes a threshold, using the node parameter's name.

The scan check uses the bag's `bag.scan_topic` (default `/scan`) from the build configuration. It
writes one CSV row per request. `evaluate_poses --queries` scores those rows against a simulator's
ground truth:

```bash
landfall query site.landfall.db --bag other_session_bag --config build.yaml \
  --every 10 --frames 3 --output results.csv
evaluate_poses --bag other_session_bag --world-to-map 8.015 0.023 0.105 --queries results.csv
evaluate_scan_check --bag other_session_bag --map site.yaml --world-to-map 8.015 0.023 0.105
```

## Accuracy so far (simulation, M4; re-run in M5 and M7)

These runs used the Gazebo depot drives from M2 (run 1 and run 2, about 9 minutes each, both
directions). Each run's images, one per second, were queried against the other run's database, with
the default thresholds and the LiDAR check. Success means within 0.25 m and 5°; wrong means more
than 0.5 m or 10° off. As in the node, a frame that finds nothing doesn't end a request: the next
image is tried until `query.timeout`. Run 1's 448 images, for example, made 440 single-frame
requests.

| Queries → database | Request | Success | AMBIGUOUS | NO_MATCH | REJECTED_CONSISTENCY | Wrong poses | Error of published poses (median / p95) |
|---|---|---|---|---|---|---|---|
| run 1 → run 2 | 1 frame | 434 / 440 (98.6 %) | 6 | 0 | 0 | 0 | 1.5 cm / 4.1 cm |
| run 1 → run 2 | 3 frames | 142 / 149 (95.3 %) | 6 | 1 | 0 | 0 | 1.3 cm / 3.5 cm |
| run 2 → run 1 | 1 frame | 425 / 431 (98.6 %) | 6 | 0 | 0 | 0 | 1.2 cm / 3.5 cm |
| run 2 → run 1 | 3 frames | 141 / 148 (95.3 %) | 6 | 1 | 0 | 0 | 1.1 cm / 2.4 cm |
| fixture (non-keyframe images) | 1 frame | 25 / 27 (92.6 %) | 0 | 2 | 0 | 0 | 1.4 cm / 4.3 cm |

(M7, with the scan alternatives check. Before it, in M4 and M5, the same requests succeeded as
often, but each single-frame direction published one wrong pose, see below.)

Re-run after the warehouse fixes (PLAN V56-V59, 2026-11-13; databases built from the two drives'
bags with scans and odometry), single-frame requests: run 1 → run 2, 438 of 444 (98.6 %), 6
`AMBIGUOUS`, no wrong pose, error 1.4 cm / 3.8 cm (median / p95); run 2 → run 1, 428 of 436
(98.2 %), 8 `AMBIGUOUS`, no wrong pose, 1.3 cm / 3.3 cm. These errors are measured in the map frame
the scan-matched databases realize, (8.030, 0.041, 0.105°) from the world, which differs by 2.3 cm
from the one fitted to AMCL's poses (8.015, 0.023, 0.105°, as used above); against that one the
medians are 2.8 cm and 2.6 cm.

The depot looks the same in its two halves when rotated 180° about the world origin. With vision
alone (M3), 19 to 20 frames per direction were ambiguous, and 1 of 156 three-frame requests
published the mirror pose, 27.8 m off. The LiDAR check rejects most mirror candidates, so far fewer
frames are ambiguous. Each direction still had one frame whose mirror pose passed every check of
that frame: the scan fit the map there, and the right place had failed. In M4 and M5 that was a
wrong single-frame pose; the 3-frame request containing it answered `REJECTED_CONSISTENCY`, as the
frames disagreed with odometry by about 27 m and 178°. Since M7 the scan alternatives check finds
that the scan fits the right place better, and the frame is `AMBIGUOUS`. A robot that stands still
gives no such odometry check (every frame finds the mirror place alike): the kidnap benchmark found
two such wrong poses before the scan alternatives check (`docs/benchmark.md`). Requests still use 3
frames by default.

The LiDAR check alone, at every scan of the two drives and of the fixture:

| Recording | Scans | True poses accepted | Wrong poses (0.5–5 m, up to 15° off) accepted |
|---|---|---|---|
| run 1 | 5,323 | 5,322 (100.0 %) | 0 of 106,460 |
| run 2 | 5,339 | 5,330 (99.8 %) | 0 of 106,780 |
| fixture | 143 | 143 (100 %) | 0 of 2,860 |

The fixture's misses are on the short leg of its L, which only three keyframes per direction cover.
Localization takes 49 to 51 ms per frame (median; p95 67 to 71 ms), LiDAR checks included, on the
development laptop (i7-13620H, 16 threads). The scan alternatives check adds about 18 ms: the same
requests took 31 to 32 ms before it (M5).
