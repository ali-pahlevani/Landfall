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
   base height and tilt, and a free map cell under the base.
4. Check the LiDAR scan nearest the image against the map at the pose (`scan_check.*`, below).
5. Cluster the poses that passed every check. A second, separate place that passed them too, with
   at least `sanity.min_inliers` inliers, makes the frame ambiguous. The default
   `ambiguity.max_score_ratio: 0` counts it whatever its score: in a building whose halves look
   alike, the wrong half can score higher than the right one.

The decision:
- Any ambiguous frame makes the request **`AMBIGUOUS`**.
- Too few valid frames give the deepest failure reached: **`NO_MATCH`** (no pose with
  `sanity.min_inliers`), then **`REJECTED_SANITY`**, then **`REJECTED_SCAN_CHECK`**.
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

`scan_check.mode` is `required` by default: no request runs without a scan. `optional` checks a
pose when a scan is there and accepts it on the vision checks otherwise. `disabled` never checks.
Use either only on a robot without a LiDAR the map matches: the LiDAR check is what stops a place
that merely looks right.

The check works on its own data too. `evaluate_scan_check` runs it on a recording with ground truth:
at every scan's true pose, where it must pass, and at wrong poses 0.5 to 5 m away, where it must
fail (*Accuracy* below).

## Automatic relocalization

With `auto_trigger.enabled: true`, the node watches the localization's own uncertainty on
`auto_trigger.pose_topic` (AMCL's `amcl_pose`). If the reported standard deviation stays above
`auto_trigger.max_std_xy` or `auto_trigger.max_std_yaw_deg` for `auto_trigger.duration` seconds,
it runs a request on its own (trigger `auto`). After that it waits at least `auto_trigger.cooldown`
seconds before the next one. It never starts while another request runs.

AMCL publishes `amcl_pose` only after a filter update, so a robot that stands still keeps its last
estimate. An AMCL that never received a pose publishes nothing, so this trigger stays quiet: that
case is what startup relocalization is for.

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
runs and the monitor's requests. Each row holds the status, the pose, and the localization's own
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
  monitor, and the last result.
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

## Accuracy so far (simulation, M4; re-run in M5)

These runs used the Gazebo depot drives from M2 (run 1 and run 2, about 9 minutes each, both
directions). Each run's images, one per second, were queried against the other run's database, with
the default thresholds and the LiDAR check. Success means within 0.25 m and 5°; wrong means more
than 0.5 m or 10° off. As in the node, a frame that finds nothing doesn't end a request: the next
image is tried until `query.timeout`. Run 1's 448 images, for example, made 440 single-frame
requests.

| Queries → database | Request | Success | AMBIGUOUS | NO_MATCH | REJECTED_CONSISTENCY | Wrong poses | Error of published poses (median / p95) |
|---|---|---|---|---|---|---|---|
| run 1 → run 2 | 1 frame | 434 / 440 (98.6 %) | 5 | 0 | 0 | 1 | 1.5 cm / 4.1 cm |
| run 1 → run 2 | 3 frames | 142 / 148 (95.9 %) | 5 | 0 | 1 | 0 | 1.3 cm / 3.5 cm |
| run 2 → run 1 | 1 frame | 425 / 431 (98.6 %) | 5 | 0 | 0 | 1 | 1.2 cm / 3.6 cm |
| run 2 → run 1 | 3 frames | 140 / 147 (95.2 %) | 5 | 1 | 1 | 0 | 1.1 cm / 2.5 cm |
| fixture (non-keyframe images) | 1 frame | 25 / 27 (92.6 %) | 0 | 2 | 0 | 0 | 1.4 cm / 4.3 cm |

The depot looks the same in its two halves when rotated 180° about the world origin. With vision
alone (M3), 19 to 20 frames per direction were ambiguous, and 1 of 156 three-frame requests
published the mirror pose, 27.8 m off. The LiDAR check rejects most mirror candidates, so far fewer frames are
ambiguous. Each direction still had one frame whose mirror pose passed every check of that frame:
the scan fit the map there, and the right place had failed. That is the single-frame wrong pose in
the table. In both cases the 3-frame request containing it answered `REJECTED_CONSISTENCY`: the
frames disagreed with odometry by about 27 m and 178°. That is why requests use 3 frames by
default.

The LiDAR check alone, at every scan of the two drives and of the fixture:

| Recording | Scans | True poses accepted | Wrong poses (0.5–5 m, up to 15° off) accepted |
|---|---|---|---|
| run 1 | 5,323 | 5,322 (100.0 %) | 0 of 106,460 |
| run 2 | 5,339 | 5,330 (99.8 %) | 0 of 106,780 |
| fixture | 143 | 143 (100 %) | 0 of 2,860 |

The fixture's misses are on the short leg of its L, which only three keyframes per direction cover.
Localization takes 31 to 32 ms per frame (median), LiDAR check included, on the development
laptop (i7-13620H).
