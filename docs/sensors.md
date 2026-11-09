# Sensors

What Landfall needs from the robot, and how to set it up. The recorder and the relocalizer read
the same inputs (`docs/recording.md`, `docs/relocalization.md`).

| Input | Needed for | Notes |
|---|---|---|
| Camera images, `sensor_msgs/Image` or `CompressedImage` | everything | one camera per database |
| `sensor_msgs/CameraInfo` | everything | calibrated `K` and `D` |
| TF `odom → base` | everything | the robot's odometry, the same transform AMCL uses |
| TF `base → camera` | everything | looked up at each image's stamp |
| TF `map → odom` | recording, the monitors | from AMCL (or slam_toolbox in localization mode) |
| `sensor_msgs/LaserScan` | relocalization | the scan check; the scan AMCL uses |
| `sensor_msgs/Imu` with orientation | optional | gravity on ramps, the tilt check |

## Camera

- **Calibration.** `CameraInfo` must hold the calibrated intrinsics and distortion: `plumb_bob`,
  `rational_polynomial` or `equidistant`. Landfall uses the images unrectified and undistorts
  points itself. A database is bound to its camera: the relocalizer refuses a camera of another
  size (`NOT_READY`). Rebuild the database after a recalibration: its landmarks were triangulated
  with the old intrinsics.
- **Mount.** Fix the camera to the chassis, high and looking along the direction of travel,
  where loads don't cover it. A camera on a moving part (a lift mast, a fork carriage) works if
  the joint is in TF: `base → camera` is looked up at each image's stamp, never assumed static. The
  startup extrinsics check then has nothing to compare and skips, and the extrinsics monitor
  covers the camera.
- **Extrinsics.** `base → camera` comes from the URDF. Measure it: the relocalizer checks the
  camera's height and tilt against it, and the map's landmarks are placed with it. A camera that
  turned or slid on its mount shows up on `/diagnostics` (the extrinsics monitor,
  `docs/relocalization.md`); the fix is the URDF, never Landfall.
- **Time stamps.** Images should be stamped at exposure. Drivers that stamp when they publish are
  late by the processing and transfer time: set `image.time_offset` (seconds added to every image
  stamp) to minus that latency, the same value in the recorder and the relocalizer. A wrong
  offset turns into position errors while the robot moves; the recorder's speed gate bounds them.
- **Transport.** `image.transport: compressed` subscribes to `<image.topic>/compressed` (JPEG or
  PNG); the simulation's recordings and runs use it. Other image_transport plugins can be
  republished to raw. Heavy JPEG compression costs features.

## Odometry

Landfall reads odometry only from the `odom → base` transform, the one AMCL uses; it subscribes to
no odometry topic. It needs it to compare the frames of a request with each other (a robot that
moved between them), to carry the pose found at an image to the moment it is published, and to
move a scan to an image's time.

The recommended source is robot_localization's `ekf_node` (EKF) fusing the wheel odometry with the
IMU's yaw rate: Nav2's documented odometry smoothing, available on Humble, Jazzy and Lyrical, light
enough for a small PC. The IMU's yaw rate keeps turns on the spot consistent, which the
relocalization's look-around makes often; wheels slip there. A minimal configuration (adapt the
topics and the frames):

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    publish_tf: true
    world_frame: odom
    odom_frame: odom
    base_link_frame: base_footprint
    odom0: wheel/odometry
    #             x      y      z      roll   pitch  yaw    vx    vy    vz     vroll  vpitch vyaw  ax     ay     az
    odom0_config: [false, false, false, false, false, false, true, true, false, false, false, true, false, false, false]
    imu0: imu/data
    imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
```

Wheel odometry alone works, but turns on the spot then disagree more with vision: raise
`consistency.max_rotation_error_deg` if requests end `REJECTED_CONSISTENCY` while the robot turns.
IMU-only odometry is not supported: it has no translation, and Nav2 and AMCL need odometry anyway.

## IMU (optional)

With an IMU, Landfall takes gravity from its orientation (roll and pitch; yaw is ignored, so no
magnetometer is needed): the solver then works on ramps, the tilt check compares vision with the
IMU, and the recorder skips keyframes on ramps (`gating.max_tilt_deg`; the relocalizer's
`gravity.ramp_tilt_deg` says when a query is on one). Without one, it assumes a flat floor
(`gravity.source: auto` falls back on its own).

- The `Imu` messages need an orientation: the driver's own filter, or a separate filter node.
  imu_tools' `imu_complementary_filter` (BSD-3) is the recommended one. `imu_filter_madgwick` is
  GPL-3.0; Landfall never links either, but mind the license of what you ship.
- TF must hold `base → imu` as a static transform. If the messages' `frame_id` is no TF frame
  (some drivers, Gazebo), set `imu.frame` to the right one.
- Gravity is used only while the robot is nearly still (`imu.max_angular_rate`) and the sample is
  fresh (`imu.max_age`): orientation filters are disturbed by acceleration.

## LiDAR

The relocalizer checks every pose against the occupancy map with a `LaserScan`
(`scan_check.topic`): use the scan AMCL uses, which matches the map. A merged scan of several
scanners works if its frame is in TF. The scan must be within `scan_check.max_time_offset` (0.5 s)
of an image; the relocalizer moves it to the image's time with odometry.

On coarse maps, `scan_check.hit_tolerance` (0.10 m) must be at least one cell: a 0.1 m map needs
0.10 to 0.15 m. A camera-only setup (`scan_check.mode: disabled`) gives up the strongest check
against wrong poses; it is an explicit opt-in (PLAN A4).

## Frames

| Frame | Parameter | Default |
|---|---|---|
| global | `frames.global` | `map` |
| odometry | `frames.odom` | `odom` |
| robot base, on the floor | `frames.base` | `base_footprint` |
| camera, optical (z forward, y down) | `frames.camera` | `CameraInfo`'s `frame_id` |

The base frame must lie on the floor (z = 0 in the map): the height check and the triangulation's
floor limit use it. A robot whose root frame sits above the floor (`base_link` on a TurtleBot, a
forklift's `chassis_link`) needs a floor frame in its URDF, or `frames.base` set to the root with
the camera height checked against it.
