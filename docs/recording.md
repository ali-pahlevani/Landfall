# Recording keyframes and building a map database

A Landfall map is built in two steps:

1. **Record** images with their camera pose in the `map` frame while the robot is well localized
   (AMCL, slam_toolbox localization, or any source of `map → odom`). Either run the recorder node
   live, or record a rosbag2 and select the keyframes from it later. Both use the same keyframe
   gates and give the same keyframes from the same data.
2. **Build** the database offline with `landfall build`, which writes the database and a report.

The relocalizer (M3) uses the database together with the occupancy map it was built for.

## What the robot must provide

| Input | Notes |
|---|---|
| Camera images, `sensor_msgs/Image` or `CompressedImage` (JPEG/PNG) | 8- or 16-bit, mono or color |
| `sensor_msgs/CameraInfo` | calibrated `K` and `D` (plumb_bob, rational_polynomial or equidistant). The images are used unrectified |
| TF `map → odom → base_footprint → camera optical frame` | base→camera is looked up at each image stamp, so a camera on a moving joint works if the joint is in TF |
| Localization covariance, `geometry_msgs/PoseWithCovarianceStamped` (`amcl_pose`) | not needed with `pose_source.type: tf_only` |
| LiDAR scan, `sensor_msgs/LaserScan` (`scan`) | stored with each keyframe: the build matches the keyframe's pose to the map with it (below). Without it the build trusts the localization's poses |
| IMU orientation, `sensor_msgs/Imu` (optional) | used only to skip keyframes on ramps |

## Keyframe gates

A frame becomes a keyframe when it passes every gate, in this order:

| Gate | Default | Counter in `RecorderStatus` |
|---|---|---|
| Moved at least `gating.min_translation` (0.5 m) or turned `gating.min_rotation_deg` (15°) since the last keyframe | | `rejected_no_motion` |
| Localization std below `gating.max_std_xy` (0.05 m, worst direction) and `gating.max_std_yaw_deg` (2°); estimate at most `pose_source.max_covariance_age` (5 s) old. By default only frames without a scan (`gating.covariance: without_scan`): the build matches the others to the map | | `rejected_pose_uncertain` |
| Speed from `odom → base` over the last `gating.speed_window` (0.2 s) below `gating.max_linear_speed` (0.6 m/s) and `gating.max_angular_speed` (0.5 rad/s) | | `rejected_speed` |
| IMU tilt below `gating.max_tilt_deg` (1.5°); with `gravity.source: imu`, no fresh IMU sample also rejects | | `rejected_tilt` |
| Sharpness (variance of the Laplacian, image scaled to at most 640 px wide) above `quality.min_sharpness` (60) | | `rejected_blur` |
| Mean intensity in [`quality.min_mean_intensity`, `quality.max_mean_intensity`] ([30, 225]); at most `quality.max_saturated_fraction` (20 %) of pixels at 250 or brighter | | `rejected_exposure` |

Frames without a pose at their stamp (TF missing) count as `rejected_tf`. The motion gate runs
first, so the other counters tell how many *due* keyframes each gate cost.

**AMCL's covariance.** With the scan recorded (`scan.topic`) this gate only holds back frames
without a scan; the build checks the others against the map. AMCL's covariance is the spread of its
particles, which is not its error. In
the M2 simulation, AMCL reported a std of 5.3 cm / 1.9° (median) while it was actually 1.1 cm /
0.13° off. On the same 9-minute drive, the default 0.05 m / 2° gates took 134 keyframes, and
0.08 m / 3° took 411.
Compare the rejection counters with how well your robot is actually localized, and raise
`gating.max_std_xy` / `gating.max_std_yaw_deg` if they reject good frames. `landfall build` reports
the result of a recording, and the pose-prior bundle adjustment flags keyframes it had to move far.

**Turning.** A keyframe taken while turning fast inherits the localization's lag. In the
simulation, keyframes taken at 0.45-0.5 rad/s were up to 1.7° off. With the gate at 0.25 rad/s,
the worst of 419 keyframes was 1.04° off (0.42° at the 95th percentile), and keyframes taken while
turning less than 2°/s stayed within 0.51°.

## Keyframe poses

A keyframe's recorded pose is the localization's (`map → base` from TF). In the URDF Maker warehouse
(2026-11-11 analysis), AMCL was off by a median 14 cm and 44 cm at the 90th percentile, up to 80 cm
along the long aisles, while it reported a median 7.5 cm; the frames its covariance gate passed
were still off by 40 cm (p90). Triangulating landmarks 9-20 m away from such poses fails, and the
landmarks that survive inherit the error. So `landfall build` doesn't take the recorded poses as
they are:

1. **Each keyframe's scan is matched to the map** near its recorded pose (`poses.search_xy`,
   1 m; `poses.search_yaw_deg`, 6°): a correlative search on the map's likelihood field, every beam
   up to the scanner's range, refined to 2.5 mm and 0.025°. Its covariance comes from the geometry
   (each beam constrains the pose along the surface it hit: tight across a corridor, loose along
   it) plus the fit's spread over the whole search window: every pose counts with its likelihood
   (beams with heavy-tailed errors, counted as `poses.effective_beams` independent ones), so a
   second fit nearly as good tens of centimetres away (shelf legs repeating along an aisle, a
   wall the map has twice) makes the match uncertain in that direction.
2. **The match is fused with the recorded pose.** Where they agree (the difference within the
   99 % chi-square of both uncertainties, `poses.agreement_chi2`), both count, the recorded pose
   at `poses.recorded_std_xy` (2 cm) and `poses.recorded_std_yaw_deg` (0.5°) in the directions
   the scan confirms: a good localization keeps its accuracy. In a direction the scan checks less
   well (along a corridor) the recorded pose counts less, its std growing with the square of the
   scan's there (scan std² / `poses.recorded_std_xy`, at most `poses.disagreement_std_xy`, 0.3 m),
   so a localization drifting along an aisle doesn't pull the pose its way. Where the scan
   contradicts it (a localization
   that drifted), the recorded pose counts only with `poses.disagreement_std_*`, which fills in
   what the scan can't see. A scan that doesn't fit (`poses.min_hit_ratio`, 60 % of its beams
   within 10 cm of an obstacle, or its best fit on the edge of the search window) confirms nothing:
   the recorded pose then counts only as loosely as a contradicted one, the odometry carries the
   keyframe, and it is flagged.
3. **Odometry ties consecutive keyframes** of a session (`odom → base` at their stamps, std
   `poses.odom_std_xy` 2 cm + 3 % of the distance and `poses.odom_std_yaw_deg` 0.5° + 10 % of the
   turn, robust to a wheel slip). A pose graph of these priors and links gives the poses that pair
   selection, triangulation and the bundle adjustment start from; along a corridor, where the
   scans can't tell, the odometry carries the corridor's ends inward.
4. The **bundle adjustment** keeps the same priors and odometry and adds the landmarks; a keyframe
   may move up to three of its prior's standard deviations (at least `ba.max_shift_xy`).

The build report's "Keyframe poses" section counts the matched keyframes, the recorded poses the
scan contradicted, the odometry links, and how far the poses moved. Older sessions without scans
(staging format 1) and bags without a scan topic build as before, from the recorded poses alone
(odometry still links them when there is some).

**Limit: places a scan can't tell apart.** Where one scan fits two positions equally well and the
localization sits on the wrong one, the build keeps that position: the localization slid there for
the same reason (it uses the same map and scans). In the URDF Maker warehouse this happened in the
southern aisles, where the map has a wall drawn twice and a scan fits 35-50 cm along the aisle as
well as at the truth: for 64 of the 70 keyframes matched more than 15 cm from the truth, the two
positions fit the scan within 0.6 nats of each other. Relocalizing there against such a database
gives poses off by as much (0.3-0.5 m in that warehouse), and the LiDAR check can't refuse them,
since the scan fits there too. Only odometry from a place where the position is clear can tell
the two apart, and the pose graph doesn't yet choose between them: the keyframes whose scan and
recorded pose agree on the wrong one outweigh the odometry (T-011). A gap in the recording also
breaks the odometry chain; the covariance gate no longer makes such gaps for frames with a scan.

## Live recording: the recorder node

`landfall_recorder` is a lifecycle node (also the component `landfall::ros::RecorderNode`). Every
activation opens a new session folder; deactivation closes it.

```bash
ros2 run landfall_ros recorder_node --ros-args \
  -p image.topic:=/camera/image_raw -p camera_info.topic:=/camera/camera_info \
  -p output_dir:=$HOME/landfall_sessions -p bond.enabled:=false -p autostart:=true
```

Without `autostart`, drive it with `ros2 lifecycle set /landfall_recorder configure` and then
`activate`, or put it under `nav2_lifecycle_manager` (keep `bond.enabled: true` then). Interfaces:

| Name | Type | |
|---|---|---|
| `~/status` | `landfall_msgs/RecorderStatus` | session, counts, last keyframe (latched) |
| `~/markers` | `visualization_msgs/MarkerArray` | the session's keyframes as arrows |
| `~/set_paused` | `std_srvs/SetBool` | pause while driving through areas that shouldn't be mapped |
| `/diagnostics` | | warns when no images arrive or most frames have no pose |

The parameters are read once, at startup (`read_only`); [parameters.md](parameters.md) lists all of
them with their ranges. Besides the gates above:

| Parameter | Default | |
|---|---|---|
| `output_dir`, `session_name` | `~/.local/share/landfall/sessions`, UTC start time | one folder per session; an existing name gets a `_2`, `_3`, ... suffix |
| `frames.global`, `frames.odom`, `frames.base`, `frames.camera` | `map`, `odom`, `base_footprint`, "" | "" = `camera_info`'s frame |
| `camera_info.topic`, `image.topic`, `image.transport` | `camera_info`, `image`, `raw` | `compressed` subscribes to `<image.topic>/compressed` |
| `image.time_offset` | 0 s | added to image stamps |
| `image.format`, `image.jpeg_quality` | `png`, 95 | keyframe image files |
| `pose_source.type`, `pose_source.covariance_topic` | `tf_with_covariance`, `amcl_pose` | `tf_only` for stacks without a covariance (no covariance gate) |
| `gating.covariance` | `without_scan` | which frames the covariance gate applies to: `without_scan` (frames with a scan are checked against the map by the build), `always`, `never` |
| `tf_timeout` | 0.2 s | how long an image waits for TF at its stamp |
| `scan.topic`, `scan.max_time_offset` | `scan`, 0.2 s | the scan stored with each keyframe: the one nearest its image, moved to the image's stamp with odometry; "" stores none |
| `gravity.source`, `imu.topic`, `imu.frame`, `imu.max_age` | `auto`, `imu`, "", 0.1 s | `imu.frame` overrides the messages' `frame_id` when it isn't a TF frame |
| `max_keyframes` | 0 (unlimited) | |
| `bond.enabled`, `autostart` | true, false | |

### Session folders

```
<output_dir>/<session>/
  session.yaml      format (version 2), landfall version, frames, camera model, gate counts,
                    complete flag
  keyframes.csv     one row per keyframe: stamp, base pose (x, y, yaw) and its covariance,
                    base->camera transform, image quality, IMU tilt, odometry pose (x, y, yaw)
  scans.csv         one row per keyframe with a scan: base->laser transform at the image's stamp,
                    angles, range limits and the ranges
  images/000000.png ...
```

Each image and scan is written before its keyframe's row, and every row is flushed. After a crash
the folder still reads; `complete: false` in `session.yaml` tells that the recorder didn't close
it. Format 1 folders (no odometry columns, no `scans.csv`) still build.

## Recording a bag instead

Record the same inputs and select keyframes later:

```bash
ros2 bag record -s sqlite3 /tf /tf_static /amcl_pose /scan /camera/image_raw/compressed /camera/camera_info /imu
```

(`-s mcap` works too.) The build matches each keyframe to the map with the scan nearest its image
(`bag.scan_topic`, within `bag.scan_max_time_offset`), as with the recorder's sessions. In
simulation add `--use-sim-time`.

## Building the database

```bash
ros2 run landfall_ros landfall build --map map.yaml --output site.landfall.db \
  --staging ~/landfall_sessions/2026-10-30_09-12-44 --bag drive_2 --config build.yaml
```

- Inputs: any number of `--staging` folders and `--bag` recordings, all from the same camera; each
  is a session in the database. The map is the one AMCL localized on.
- The output must not exist. The database is written under `<output>.partial` and renamed when
  complete, so a failed build leaves nothing behind.
- The report (`report.md`, `report.json`, `coverage.png`) goes to `<output>.report/`
  (`--report-dir` to change). `coverage.png` draws the keyframes over the map, one heading rose per
  1 m cell (filled sectors are the directions driven; drive every aisle both ways), and the
  landmarks.
- `--threads N` (default: all cores) doesn't change the result: the same input gives the same
  content hash.
- `--map-semantics humble|jazzy`: nav2_map_server converts color and transparent images
  differently before and after Nav2 1.3; the default is the distro the CLI was built for. The
  relocalizer compares its map with what map_server publishes, so build on the distro the robot
  runs.

The configuration file (all keys optional; `inspect` prints the effective configuration of a
database):

```yaml
backend: {local: orb, global: bow, matcher: auto}   # auto: ratio test (binary) or mutual NN (float)
features: {max_keypoints: 2000, grid: [8, 6]}       # options of the local backend
matching: {ratio: 0.8}                              # options of the matcher
vocab: {branching: 10, depth: 5}
pairs:                                    # which keyframes are matched
  max_distance: 6.0                       #   camera centers within 6 m ...
  max_angle_deg: 60.0                     #   ... looking within 60 deg of each other,
  retrieval_k: 10                         #   plus the 10 most similar by bag of words,
  max_per_keyframe: 25                    #   at most 25 proposals per keyframe
  min_inliers: 20
  max_epipolar_error_px: 4.0              # two-view verification (5-point LO-RANSAC)
  max_rotation_deviation_deg: 5.0         # relative rotation vs the recorded poses
  min_baseline: 0.05                      # closer pairs use a rotation-only model
triangulation:
  max_reprojection_px: 0                  # 0: 0.6 % of the image width
  min_angle_deg: 2.0
  min_track_length: 3
  depth_range: [0.2, 30.0]
  min_height: -0.05                       # below the floor: reflections
poses:                                    # keyframe poses (see "Keyframe poses" above)
  scan_matching: true                     # match keyframes that have a scan to the map
  search_xy: 1.0                          #   around the recorded pose, m
  search_yaw_deg: 6.0
  max_range: 0                            #   longer beams are ignored; 0: the scanner's own
  beam_sigma: 0.05                        #   likelihood field spread, m
  hit_tolerance: 0.10
  min_beams: 60
  min_hit_ratio: 0.6                      #   below: the scan doesn't match (the pose follows odometry)
  effective_beams: 40                     #   beams counted as independent in the covariance
  recorded_std_xy: 0.02                   # the recorded pose where the scan agrees with it (in
  recorded_std_yaw_deg: 0.5               #   the directions the scan confirms)
  disagreement_std_xy: 0.3                # ... and where the scan contradicts it
  disagreement_std_yaw_deg: 3.0
  agreement_chi2: 11.34                   # 99 % of a 3-DoF chi-square
  odometry: true                          # tie consecutive keyframes by their odometry
  odom_std_xy: 0.02                       #   + odom_std_xy_per_m of the distance
  odom_std_xy_per_m: 0.03
  odom_std_yaw_deg: 0.5                   #   + odom_std_yaw_per_rad of the turn
  odom_std_yaw_per_rad: 0.1
  odom_max_step: 2.0                      #   farther steps (a gap) aren't tied
  prior_loss_scale: 3.0                   # Huber scale of the scan priors, in stds
ba:                                       # bundle adjustment
  enabled: true
  max_shift_xy: 0.10                      # hard bounds on how far a keyframe may move (at least)
  max_shift_yaw_deg: 2.0
  loss_scale_px: 2.0
  prior_floor_xy: 0.02                    # added to the recorded std
  prior_floor_yaw_deg: 0.5
  prior_std_xy: 0                         # > 0: use instead of the recorded covariance
  prior_std_yaw_deg: 0
  max_iterations: 50
  max_reprojection_px: 0                  # landmark re-filter after BA; 0: 0.75 x triangulation
db: {prune_unobserved_features: true, covisibility_min_shared: 5, global_dtype: f16}
threads: 0
seed: 42
bag:                                      # where `--bag` recordings keep their data
  image_topic: /camera/image_raw          # Image or CompressedImage
  camera_info_topic: /camera/camera_info
  pose_topic: /amcl_pose
  imu_topic: ""
  imu_frame: ""
  scan_topic: /scan                       # LaserScan: keyframe poses, and `landfall query`
  scan_max_time_offset: 0.2               # farthest scan from a keyframe's image, s
  gravity_source: auto
  imu_max_age: 0.1
  pose_source: tf_with_covariance
  image_time_offset: 0.0
  speed_window: 0.2
  frames: {global: map, odom: odom, base: base_footprint, camera: ""}
gating: {...}                             # same keys and defaults as the recorder
quality: {...}
max_keyframes: 0
```

**BA priors.** A keyframe matched to the map with its scan has the fused prior above. A keyframe
without a scan keeps its recorded pose, weighed by its recorded covariance. If the localization
misreports its covariance (as AMCL does above), set `ba.prior_std_xy` / `ba.prior_std_yaw_deg` to
its actual accuracy (per axis); they apply to keyframes without a scan. With AMCL's 5 cm
reports, BA moved simulated keyframes by up to 5 cm *away* from the truth. With 2 cm, one keyframe
(the first of the drive, accelerating, its landmarks 7-20 m straight ahead) still slid 8 cm along
its weakly constrained viewing direction. With the measured 1 cm / 0.5°, the largest shift was
2.8 cm, and the yaw error fell from 1.09° to 0.43° (worst keyframe).

Other commands:

```bash
landfall inspect site.landfall.db                 # what it contains, the build configuration
landfall report site.landfall.db --output DIR     # regenerate the report
landfall verify site.landfall.db --map map.yaml   # integrity, content hash, map binding
landfall migrate site.landfall.db                 # upgrade an older schema (keeps a backup)
```

(`landfall` is `ros2 run landfall_ros landfall`.)

## The simulation recording

The depot simulation (Gazebo Harmonic, Nav2, TurtleBot 4 with a 640 x 480 camera and Gazebo's
ground-truth pose) records and evaluates the pipeline. It needs the simulation image and the Jazzy
workspace:

```bash
scripts/docker_ci.sh jazzy        # builds .docker_ws/jazzy/install
scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/record_depot.sh /data/depot_run1'
```

The script starts the simulation with GPU headless rendering, tunes AMCL for the simulation
(`sim/params/amcl_depot_sim.yaml`), runs the recorder (`sim/params/recorder_depot_sim.yaml`), and
records a bag. It then drives the route `sim/routes/depot_loop.txt` with Nav2, forwards and back
(about 230 m in 9 minutes). Results land in `.docker_ws/sim/data/depot_run1/`. Build and evaluate:

```bash
scripts/sim.sh 'ros2 run landfall_ros landfall build --map /ws/src/landfall/landfall_ros/test/fixtures/depot_small/depot.yaml \
  --bag /data/depot_run1/bag --config /ws/install/landfall_bringup/share/landfall_bringup/sim/params/build_depot_sim.yaml \
  --output /data/depot.landfall.db'
scripts/sim.sh '/ws/install/landfall_benchmark/lib/landfall_benchmark/evaluate_poses --bag /data/depot_run1/bag \
  --staging /data/depot_run1/sessions/depot --db /data/depot.landfall.db --amcl-topic /amcl_pose'
```

`evaluate_poses` compares poses with the ground truth in the map frame, twice: with the nominal
transform (the depot map is Gazebo's world shifted by +8 m in x), and with the best-fitting one (the
map is aligned to the world only to about 2 cm). The test fixture
(`landfall_ros/test/fixtures/depot_small`) is a cut of this recording.
