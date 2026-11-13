# Troubleshooting

Symptoms, their usual causes, and where to look. Every relocalization answers with a status and a
message (`~/status`, the service's answer, the node's log); `docs/relocalization.md` explains each
status and lists the `NOT_READY` reasons with their fixes.

## Recording

| Symptom | Look at | Usual cause |
|---|---|---|
| Few keyframes | the recorder's `~/status`: rejections per reason | `rejected_pose_uncertain`: a frame without a scan whose localization reports a std above `gating.max_std_xy`, often while it is right (AMCL overestimates; `docs/tuning.md`); record the scan (`scan.topic`) so the build checks poses against the map instead. `rejected_speed`: driving or turning faster than the gates. `rejected_blur`, `rejected_exposure`: blurred, dark or overexposed images. `rejected_tf`: no transform at the image |
| Keyframes in some directions only | the build report's heading rose | the lanes were driven one way; drive them both ways |
| `no transform … at the image` in the recorder's log | TF | `image.time_offset`, TF too late (`tf_timeout`), the camera frame missing |

## Building

| Symptom | Look at | Usual cause |
|---|---|---|
| Few landmarks, short tracks | the build report | too little overlap between keyframes, textureless walls, a wrong camera calibration |
| `no features were found in any keyframe`, or few features per keyframe | the build report's "Features" line: how much of the image the floor mask hid | a camera that looks down at the floor: lower `features.floor_mask_distance`, or set 0 (`docs/recording.md`) |
| Keyframes flagged at the BA bound | the build report's BA section | poses recorded while the localization was off; drop that stretch or re-record it |
| `landfall build` refuses the map | its message | the map YAML or image differs from the one the recording localized on |

## Relocalizing

| Status | Usual cause | What helps |
|---|---|---|
| `NOT_READY` | something missing: database, map, camera, scan, TF, AMCL | the message, and the table in `docs/relocalization.md` |
| `NO_MATCH` often | the robot looks at views the recording never saw, or the scene changed | record there in this direction; let the robot turn (the example tree looks around from eight headings) |
| `AMBIGUOUS` | two places pass every check: repetitive aisles, mirrored halves | by design: the robot moves on and asks again |
| `REJECTED_SCAN_CHECK` | the scan doesn't fit the map at the pose: objects the map has are gone (beams see through them), objects the map doesn't have stand within a few metres (goods, pallets: too many short beams), the map is outdated, the scan topic or laser frame is wrong, scan and image clocks differ | `evaluate_scan_check` on a recording; update the map; `scan_check.topic`, `scan_check.max_time_offset` |
| `REJECTED_CONSISTENCY` | the frames disagree with odometry: wheels slip while turning, or the frames saw different places | fuse the IMU's yaw rate (`docs/sensors.md`); `consistency.max_rotation_error_deg` |
| `REJECTED_SANITY` | height, tilt, inliers or coverage out of range | wrong extrinsics (the extrinsics monitor reports a camera that moved), a camera on a moving part without its joint in TF |
| `BUSY` | another request runs | the behavior tree node asks again; the startup relocalization may still run |

A pose is published but AMCL doesn't take it:
- AMCL must be active and subscribed: otherwise the request answers `NOT_READY`.
- AMCL's `set_initial_pose: true` makes it keep its own initial pose at activation (PLAN §7.16);
  use `always_reset_initial_pose: true` with the startup relocalization.
- AMCL's warning "Failed to transform initial pose in time" is harmless: it sets the pose as given
  (`docs/nav2_integration.md`).

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
