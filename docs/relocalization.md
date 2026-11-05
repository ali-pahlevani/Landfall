# Relocalization

`landfall_relocalizer` finds the robot on the map from camera images and hands the pose to AMCL.
It localizes each image against a map database built with `landfall build`
([recording.md](recording.md)). It also checks that the result is plausible and that several frames
agree with odometry. Only then does it publish `/initialpose`, or call AMCL's `set_initial_pose`
service.

> **Status (M3): vision checks only.** LiDAR verification of every candidate pose arrives in M4
> (PLAN §7.12). Without it, a place that looks like another one can win. In the simulated depot,
> whose two halves look identical when rotated 180°, 1 of 156 cross-session requests published the
> mirror pose, 27.8 m off (see *Accuracy* below). Until M4, run requests with `dry_run: true` and
> compare them with AMCL; don't let the node publish unattended.

## Running it

```bash
ros2 run landfall_ros relocalizer_node --ros-args \
  -p database_path:=/path/to/site.landfall.db \
  -p image.topic:=/camera/image_raw -p image.transport:=raw \
  -p camera_info.topic:=/camera/camera_info \
  -p frames.base:=base_footprint \
  -p autostart:=true
```

The node is also a component, `landfall::ros::RelocalizerNode`. It is a lifecycle node with a bond,
so it can be listed after `amcl` in the `node_names` of
`lifecycle_manager_localization`. Nav2 launch files and parameter overlays come with M6.

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
| `no transform odom -> base_footprint at the image` / `… -> camera` | TF at the image stamps (see `tf_timeout` and `image.time_offset`) |
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
4. Cluster the valid poses. If a second, separate place scores at least `ambiguity.max_score_ratio`
   of the best one, the frame is ambiguous.

The decision:
- Any ambiguous frame makes the request **`AMBIGUOUS`**.
- Too few valid frames give the deepest failure reached: **`NO_MATCH`** (no pose with
  `sanity.min_inliers`), then **`REJECTED_SANITY`**.
- Frames that disagree with the odometry between them by more than
  `consistency.max_translation_error` or `consistency.max_rotation_error_deg` give
  **`REJECTED_CONSISTENCY`**.
- Otherwise the request succeeds. The pose is moved to the latest odometry
  (`output.compensate_motion`) and published with σ = `output.std_xy` and `output.std_yaw_deg`.

Every threshold except the structural parameters can change at runtime with `ros2 param set` and
applies to the next request ([parameters.md](parameters.md)).

## Startup relocalization

With `startup.enabled: true`, the node relocalizes on its own after activation. It starts after
`startup.delay` and retries every `startup.retry_interval` until one attempt succeeds. Attempts
answered `NOT_READY` don't count; after `startup.max_attempts` attempts that ran, it gives up. Pair
it with AMCL's `always_reset_initial_pose: true` (PLAN §7.16).

## Diagnostics and debugging

- `/diagnostics`: readiness, database size and hash, the startup extrinsics check, and the last
  result.
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
carriage, skips this check. The extrinsics monitor (M4) covers that case.

**Cameras that stamp images when they publish them.** Some drivers stamp an image when they publish
it, not at exposure, so poses are looked up a few tens of milliseconds late. Set
`image.time_offset` to the negative latency, the same value as for recording.

## Offline evaluation

`landfall query` replays a bag's images through the same localizer and decision. Its options:
- `--frames N` emulates requests of N frames.
- `--every` thins the images.
- `--skip-keyframes` leaves out the database's own images.
- `--set` changes a threshold, using the node parameter's name.

It writes one CSV row per request. `evaluate_poses --queries` scores those rows against a
simulator's ground truth:

```bash
landfall query site.landfall.db --bag other_session_bag --config build.yaml \
  --every 10 --frames 3 --output results.csv
evaluate_poses --bag other_session_bag --world-to-map 8.015 0.023 0.105 --queries results.csv
```

## Accuracy so far (simulation, M3)

These runs used the Gazebo depot drives from M2 (run 1 and run 2, about 9 minutes each, both
directions). Each run's images, one per second, were queried against the other run's database.
Success means within 0.25 m and 5°; wrong means more than 0.5 m or 10° off.

| Queries → database | Request | Success | AMBIGUOUS | NO_MATCH | REJECTED_CONSISTENCY | Wrong poses | Error of published poses (median / p95) |
|---|---|---|---|---|---|---|---|
| run 1 → run 2 | 1 frame | 419 / 449 (93.3 %) | 20 | 8 | 0 | 2 | 1.5 cm / 4.2 cm |
| run 1 → run 2 | 3 frames | 138 / 160 (86.2 %) | 20 | 1 | 1 | 0 | 1.3 cm / 3.5 cm |
| run 2 → run 1 | 1 frame | 409 / 439 (93.2 %) | 19 | 8 | 0 | 3 | 1.2 cm / 3.6 cm |
| run 2 → run 1 | 3 frames | 135 / 156 (86.5 %) | 19 | 1 | 0 | 1 | 1.1 cm / 2.8 cm |
| fixture (non-keyframe images) | 1 frame | 25 / 35 (71.4 %) | 0 | 10 | 0 | 0 | 2.1 cm / 3.3 cm |

The depot looks the same in its two halves when rotated 180° about the world origin. Every
`AMBIGUOUS` result and every wrong pose is that mirror place. In 8 of the 20 ambiguous frames, the
wrong place even had more inliers than the right one. Multi-frame consistency can't separate the two
places, because odometry agrees in both. The occupancy map is not symmetric: only 27 % of occupied
cells have an obstacle within 0.10 m of their mirror. So the LiDAR check of M4 is the safeguard
designed for this case.

The fixture's misses are on the short leg of its L, which only three keyframes per direction cover.
Localization takes 30 ms per frame (median) on the development laptop (i7-13620H).
