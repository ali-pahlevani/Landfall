# Parameters

<!-- Generated from the nodes' parameter declarations by scripts/update_parameters_doc.sh. Do not edit: test_parameters_doc fails when this file is stale. -->

Parameters marked *at startup* are read-only: set them in the launch file or YAML. The others can change with `ros2 param set` and apply to the next request (relocalizer) or frame (recorder). Every value is checked against its range on startup and on every change.

## landfall_relocalizer

Relocalizer node (`relocalizer_node`, component `landfall::ros::RelocalizerNode`); see [relocalization.md](relocalization.md).

| Name | Type | Default | Range | Changes | Description |
|---|---|---|---|---|---|
| `ambiguity.cluster_angle_deg` | double | `15` | [1, 180] | at runtime | Poses this close (and within cluster_radius) are one place, degrees |
| `ambiguity.cluster_radius` | double | `0.5` | [0.05, 5] | at runtime | Poses this close (and within cluster_angle_deg) are one place, metres |
| `ambiguity.max_score_ratio` | double | `0.5` | [0, 1] | at runtime | Runner-up / best inliers at or above this is AMBIGUOUS |
| `autostart` | bool | `false` |  | at startup | relocalizer_node executable: configure and activate on start |
| `backend.num_threads` | integer | `0` | [0, 64] | at startup | Threads solving places in parallel; 0: hardware concurrency |
| `bond.enabled` | bool | `true` |  | at startup | Heartbeat bond with nav2_lifecycle_manager |
| `camera_info.topic` | string | `"camera_info"` |  | at startup | sensor_msgs/CameraInfo topic |
| `consistency.max_rotation_error_deg` | double | `5` | [0.5, 45] | at runtime | Largest disagreement with odometry between frames, degrees |
| `consistency.max_translation_error` | double | `0.15` | [0.01, 2] | at runtime | Largest disagreement with odometry between frames, metres |
| `consistency.min_frame_interval` | double | `0.3` | [0, 5] | at runtime | Seconds between the frames used |
| `consistency.num_frames` | integer | `3` | [1, 10] | at runtime | Frames that must agree |
| `database_path` | string | `""` |  | at startup | Map database (*.landfall.db) built by `landfall build` |
| `debug.dump_dir` | string | `""` |  | at runtime | If set, every query frame and its result are saved there for tuning |
| `debug.publish_image` | bool | `true` |  | at runtime | Publish ~/debug_image (only rendered with subscribers) |
| `debug.publish_markers` | bool | `true` |  | at runtime | Publish ~/markers |
| `extrinsics_check.warn_rotation_deg` | double | `0.5` | [0.1, 10] | at startup | Startup check: warn if base->camera differs from the database's by more degrees |
| `extrinsics_check.warn_translation` | double | `0.01` | [0.001, 1] | at startup | Startup check: warn if base->camera differs from the database's by more metres |
| `frames.base` | string | `"base_footprint"` |  | at startup | Robot base frame on the floor |
| `frames.camera` | string | `""` |  | at startup | Camera optical frame; empty: camera_info's frame_id |
| `frames.global` | string | `"map"` |  | at startup | Global frame of the published pose |
| `frames.odom` | string | `"odom"` |  | at startup | Odometry frame (motion between frames) |
| `gravity.ramp_tilt_deg` | double | `1.5` | [0.5, 15] | at runtime | IMU tilt above which the robot is on a ramp, degrees |
| `gravity.source` | string | `"auto"` |  | at startup | auto (IMU when fresh), imu (required) or flat_floor |
| `image.time_offset` | double | `0` | [-0.5, 0.5] | at startup | Seconds added to image stamps |
| `image.topic` | string | `"image"` |  | at startup | Image topic (base name for compressed) |
| `image.transport` | string | `"raw"` |  | at startup | raw (sensor_msgs/Image) or compressed (<topic>/compressed) |
| `imu.frame` | string | `""` |  | at startup | IMU frame in TF; empty: the messages' frame_id |
| `imu.max_age` | double | `0.1` | [0.01, 1] | at runtime | Oldest usable IMU sample, seconds |
| `imu.max_angular_rate` | double | `0.1` | [0, 2] | at runtime | IMU gravity is used only below this turn rate, rad/s |
| `imu.topic` | string | `"imu"` |  | at startup | sensor_msgs/Imu with orientation |
| `map_yaml` | string | `""` |  | at startup | Optional map_server YAML; its file hashes must equal the database's |
| `matching.min_cosine` | double | `0.82` | [0, 1] | at runtime | Smallest cosine for float descriptors (mutual nearest) |
| `matching.ratio` | double | `0.8` | [0.5, 1] | at runtime | Lowe ratio for binary descriptors (with a mutual check) |
| `output.amcl_node` | string | `"amcl"` |  | at startup | AMCL node that must be active before publishing; empty: don't check |
| `output.compensate_motion` | bool | `true` |  | at runtime | Move the pose to the latest odometry before publishing |
| `output.mode` | string | `"topic"` |  | at startup | topic (publish output.topic) or service (call AMCL's output.service) |
| `output.service` | string | `"/set_initial_pose"` |  | at startup | nav2_msgs/srv/SetInitialPose service |
| `output.std_xy` | double | `0.25` | [0.01, 5] | at runtime | Standard deviation in x and y of the published pose, metres |
| `output.std_yaw_deg` | double | `5` | [0.1, 90] | at runtime | Standard deviation in yaw of the published pose, degrees |
| `output.topic` | string | `"/initialpose"` |  | at startup | geometry_msgs/PoseWithCovarianceStamped topic |
| `pnp.max_iterations` | integer | `10000` | [100, 1000000] | at runtime | RANSAC iterations |
| `pnp.max_reproj_error_px` | double | `0` | [0, 50] | at runtime | RANSAC inlier threshold in pixels; 0: 1 % of the image width |
| `pnp.solver` | string | `"upright_then_p3p"` |  | at runtime | upright, p3p, upright_then_p3p or opencv |
| `pnp.success_prob` | double | `0.9999` | [0.9, 0.999999] | at runtime | RANSAC confidence |
| `query.timeout` | double | `10` | [1, 120] | at runtime | Seconds for a whole request, frame collection included |
| `query.top_k` | integer | `10` | [1, 100] | at runtime | Keyframes retrieved per frame |
| `require_map_match` | bool | `true` |  | at startup | Refuse requests unless the map on 'map' equals the database's occupancy map |
| `sanity.max_floor_elevation` | double | `0` | [0, 5] | at runtime | Extra height allowed on a ramp, metres; 0: refuse ramp queries |
| `sanity.max_height_error` | double | `0.1` | [0.01, 1] | at runtime | Largest base height above the floor, metres |
| `sanity.max_mean_reproj_error_px` | double | `0` | [0, 20] | at runtime | Largest mean inlier reprojection error; 0: half the PnP threshold |
| `sanity.max_tilt_error_deg` | double | `3` | [0.5, 30] | at runtime | Largest roll/pitch difference to gravity, degrees |
| `sanity.min_inlier_coverage` | double | `0.15` | [0, 1] | at runtime | Smallest fraction of the 8x6 image grid holding inliers |
| `sanity.min_inlier_ratio` | double | `0.25` | [0, 1] | at runtime | Smallest inlier ratio |
| `sanity.min_inliers` | integer | `40` | [6, 10000] | at runtime | Fewest PnP inliers |
| `sanity.require_free_space` | bool | `true` |  | at runtime | The base must stand on a known-free map cell |
| `startup.delay` | double | `2` | [0, 60] | at startup | Seconds after activation before the first attempt |
| `startup.enabled` | bool | `false` |  | at startup | Relocalize once after activation, when ready |
| `startup.max_attempts` | integer | `3` | [1, 100] | at startup | Attempts that ran (not counting NOT_READY) before giving up |
| `startup.retry_interval` | double | `5` | [0, 300] | at startup | Seconds between startup attempts |
| `tf_timeout` | double | `0.2` | [0, 5] | at runtime | Seconds to wait for TF at an image stamp |

## landfall_recorder

Keyframe recorder (`recorder_node`, component `landfall::ros::RecorderNode`); see [recording.md](recording.md).

| Name | Type | Default | Range | Changes | Description |
|---|---|---|---|---|---|
| `autostart` | bool | `false` |  | at startup | recorder_node executable: configure and activate on start |
| `bond.enabled` | bool | `true` |  | at startup | Heartbeat bond with nav2_lifecycle_manager |
| `camera_info.topic` | string | `"camera_info"` |  | at startup | sensor_msgs/CameraInfo topic |
| `frames.base` | string | `"base_footprint"` |  | at startup | Robot base frame on the floor |
| `frames.camera` | string | `""` |  | at startup | Camera optical frame; empty: camera_info's frame_id |
| `frames.global` | string | `"map"` |  | at startup | Global (map) frame of the keyframe poses |
| `frames.odom` | string | `"odom"` |  | at startup | Odometry frame (speed gate) |
| `gating.max_angular_speed` | double | `0.5` | [0, 5] | at startup | Fastest turn for a keyframe, rad/s |
| `gating.max_linear_speed` | double | `0.6` | [0, 5] | at startup | Fastest speed for a keyframe, m/s |
| `gating.max_std_xy` | double | `0.05` | [0.001, 1] | at startup | Largest localization std in x/y, metres |
| `gating.max_std_yaw_deg` | double | `2` | [0.1, 45] | at startup | Largest localization std in yaw, degrees |
| `gating.max_tilt_deg` | double | `1.5` | [0.5, 15] | at startup | IMU tilt above which no keyframe is taken |
| `gating.min_rotation_deg` | double | `15` | [0, 180] | at startup | A keyframe needs this many degrees since the last one, or gating.min_translation |
| `gating.min_translation` | double | `0.5` | [0, 10] | at startup | A keyframe needs this many metres since the last one, or gating.min_rotation_deg |
| `gating.speed_window` | double | `0.2` | [0.05, 2] | at startup | Seconds of odometry for the speed gate |
| `gravity.source` | string | `"auto"` |  | at startup | auto (IMU when fresh), imu (required) or flat_floor |
| `image.format` | string | `"png"` |  | at startup | Keyframe image files: png or jpg |
| `image.jpeg_quality` | integer | `95` | [50, 100] | at startup | JPEG quality when image.format is jpg |
| `image.time_offset` | double | `0` | [-0.5, 0.5] | at startup | Seconds added to image stamps |
| `image.topic` | string | `"image"` |  | at startup | Image topic (base name for compressed) |
| `image.transport` | string | `"raw"` |  | at startup | raw (sensor_msgs/Image) or compressed (<topic>/compressed) |
| `imu.frame` | string | `""` |  | at startup | IMU frame in TF; empty: the messages' frame_id |
| `imu.max_age` | double | `0.1` | [0.01, 1] | at startup | Oldest usable IMU sample, seconds |
| `imu.topic` | string | `"imu"` |  | at startup | sensor_msgs/Imu with orientation |
| `max_keyframes` | integer | `0` | [0, 10000000] | at startup | Stop after this many keyframes; 0: unlimited |
| `output_dir` | string | `"~/.local/share/landfall/sessions"` |  | at startup | Directory that receives one staging folder per recording session |
| `pose_source.covariance_topic` | string | `"amcl_pose"` |  | at startup | geometry_msgs/PoseWithCovarianceStamped of the localization |
| `pose_source.max_covariance_age` | double | `5` | [0, 60] | at startup | Oldest usable covariance estimate, seconds |
| `pose_source.type` | string | `"tf_with_covariance"` |  | at startup | tf_with_covariance (gate on the localization covariance) or tf_only |
| `quality.max_mean_intensity` | integer | `225` | [0, 255] | at startup | Brightest mean intensity |
| `quality.max_saturated_fraction` | double | `0.2` | [0, 1] | at startup | Largest fraction of pixels at 250 or brighter |
| `quality.min_mean_intensity` | integer | `30` | [0, 255] | at startup | Darkest mean intensity |
| `quality.min_sharpness` | double | `60` | [0, 100000] | at startup | Smallest variance of the Laplacian (images at most 640 px wide) |
| `session_name` | string | `""` |  | at startup | Name of the session folder; empty: the UTC start time |
| `tf_timeout` | double | `0.2` | [0, 5] | at startup | Seconds to wait for TF at an image stamp |
