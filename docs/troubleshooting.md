# Troubleshooting

Symptoms, their usual causes, and where to look. Every relocalization answers with a status and a
message (`~/status`, the service's answer, the node's log); `docs/relocalization.md` explains each
status and lists the `NOT_READY` reasons with their fixes.

## Recording

| Symptom | Look at | Usual cause |
|---|---|---|
| Few keyframes | the recorder's `~/status`: rejections per reason | `rejected_pose_uncertain`: a frame without a scan whose localization reports a std above `gating.max_std_xy`, often while it is right (AMCL overestimates; `docs/tuning.md`); record the scan (`scan.topic`) so the build checks poses against the map instead. `rejected_speed`: driving or turning faster than the gates. `rejected_blur`, `rejected_exposure`: blurred, dark or overexposed images. `rejected_tf`: no transform at the image |
| Warning: only part of the floor the robot can reach has a view within 1 m | the coverage map: red has no view within 1 m, yellow one direction | floor never driven, or driven only past views with few landmarks: drive the red parts both ways in another session and build from all sessions (`docs/recording.md`, "Where to drive"). Floor the robot never uses (fixed lanes) can stay red: set `coverage.target` to 0. Yellow that stays yellow faces open floor or distant walls |
| `no transform … at the image` in the recorder's log | TF | `image.time_offset`, TF too late (`tf_timeout`), the camera frame missing |

## Building

| Symptom | Look at | Usual cause |
|---|---|---|
| Few landmarks, short tracks | the build report | too little overlap between keyframes, textureless walls, a wrong camera calibration |
| `no features were found in any keyframe`, few features per keyframe, or the warning that the floor mask hides most of the image | the build report's "Features" line: how much of the image the floor mask hid | a camera that looks down at the floor: lower `features.floor_mask_distance`, or set 0 (`docs/recording.md`) |
| Keyframes flagged at the BA bound | the build report's BA section | poses recorded while the localization was off; drop that stretch or re-record it |
| `landfall build` refuses the map | its message | the map YAML or image differs from the one the recording localized on |
| Warning: scans that fit a second place almost as well | the coverage map: orange keyframes | the map draws a wall twice there (a SLAM map's loop closure), or the structure repeats: relocalization there can be off by the distance it names. Fix the map (re-map that area, or erase the doubled wall) |
| Warning: turning odometry errs more than `poses.odom_std_xy_per_rad` | the report's "Odometry against the scans" | the wheels slip in turns (skid steering): set `poses.odom_std_xy_per_rad` to the measured value (`docs/tuning.md`) |

## Relocalizing

| Status | Usual cause | What helps |
|---|---|---|
| `NOT_READY` | something missing: database, map, camera, scan, TF, AMCL | the message, and the table in `docs/relocalization.md` |
| `NO_MATCH` often | the robot looks at views the recording never saw, or the scene changed | record there in this direction; let the robot turn (the example tree looks around from eight headings) |
| `AMBIGUOUS` | two places pass every check: repetitive aisles, mirrored halves | by design: the robot moves on and asks again |
| `AMBIGUOUS`: `the landmarks come from keyframes whose LiDAR scans fitted two places within 1 m` | the map draws a wall twice (or the structure repeats) where the recording drove; the build warned that keyframes `fit a second place` (orange on its coverage map) | fix the map there and rebuild; elsewhere the robot relocalizes |
| `REJECTED_SCAN_CHECK` with `inliers < 40 where the scan …` (the best hypothesis) | vision found the place with few landmarks, and the LiDAR can't confirm it alone: the scan fits another place, or a place within 1 m, almost as well | let the robot turn and ask again; record more views there |
| `REJECTED_SCAN_CHECK` | the scan doesn't fit the map at the pose: objects the map has are gone (beams see through them), objects the map doesn't have stand within a few metres (goods, pallets: too many short beams), the map is outdated, the scan topic or laser frame is wrong, scan and image clocks differ | `evaluate_scan_check` on a recording; update the map; `scan_check.topic`, `scan_check.max_time_offset` |
| `REJECTED_CONSISTENCY` | the frames disagree with odometry: wheels slip while turning, or the frames saw different places | fuse the IMU's yaw rate (`docs/sensors.md`); `consistency.max_rotation_error_deg` |
| `REJECTED_SANITY` | height, tilt, inliers or coverage out of range | wrong extrinsics (the extrinsics monitor reports a camera that moved), a camera on a moving part without its joint in TF |
| `REJECTED_SANITY` with `the inliers fix no pose (their geometry is degenerate)` | the inliers lie so that no pose fits them better than another (e.g. all on one line of sight) | nothing to change: another frame or heading usually has better ones |
| `BUSY` | another request runs | the behavior tree node asks again; the startup relocalization may still run |

A pose is published but AMCL doesn't take it:
- AMCL must be active and subscribed: otherwise the request answers `NOT_READY`.
- AMCL's `set_initial_pose: true` makes it keep its own initial pose at activation (PLAN §7.16);
  use `always_reset_initial_pose: true` with the startup relocalization.
- AMCL's warning "Failed to transform initial pose in time" is harmless: it sets the pose as given
  (`docs/nav2_integration.md`).

AMCL takes the pose and then settles a few decimetres away: its particles came from the published
covariance (`docs/relocalization.md`, *The published covariance*). If the place it settles is the
right one, the covariance was too narrow for this map: measure it (`docs/tuning.md`) and raise
`output.map_std_xy`, or publish the fixed values (`output.covariance: fixed`). If AMCL drifts off
a right pose, look at its own parameters (motion noise, laser model) first.

## Nav2

| Symptom | Usual cause |
|---|---|
| The tree doesn't load: unknown node `ReLocalize` | `landfall_nav2_bt_nodes` missing from bt_navigator's `plugin_lib_names`. On Humble, a parameter file that lists the plugins must list it too (`docs/nav2_integration.md`) |
| `ReLocalize` fails with `UNAVAILABLE` | no relocalizer service under `service_name`: the relocalizer isn't active, or runs in another namespace |
| `ReLocalize` fails with `TIMEOUT` | the relocalizer answered after `server_timeout`; it must exceed the request's `timeout` (the default does) |
| NavigateToPose goals rejected at startup | Nav2 1.2 and later refuse goals while AMCL has no pose: the relocalizer's startup relocalization gives the first one (`startup_relocalization`) |
| The robot drives on after a kidnap | Nav2 notices a wrong pose only when the robot can't move: the kidnap watchdog (`docs/relocalization.md`) |

## Monitors

| Symptom | Usual cause |
|---|---|
| `/diagnostics` warns: extrinsics monitor | the camera moved on its mount (or its URDF is wrong); correct the URDF. Below `extrinsics_monitor.min_samples` it only collects |
| The extrinsics monitor never samples | the localization's std never drops below `extrinsics_monitor.max_amcl_std_xy`, or the robot is never slow |
| The kidnap watchdog relocalizes a right localization | the shadow log shows the checks that disagreed; raise `kidnap_watchdog.confirmations` or the limits, and report it: it means vision passed every check at a wrong place |

## Simulation

| Symptom | Usual cause |
|---|---|
| No camera images in Gazebo | Gazebo started without a display and without `--headless-rendering` (the scripts in `landfall_bringup/sim` add it) |
| The first run hangs on downloading | Gazebo fetches the depot from Fuel once into `.docker_ws/sim/fuel` |
| A lifecycle manager stuck at "Configuring map_server" | a service answer lost at startup, a DDS race; Nav2's lifecycle manager has no timeout for it. Restart the simulation (the benchmark script restarts on its own) |
