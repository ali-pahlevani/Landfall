# Landfall

**Monocular visual relocalization for Nav2, on the 2D maps you already have.**

Robots running AMCL need an initial pose at startup and after being moved ("kidnapped"). AMCL's
global localization struggles where laser scans look alike (identical warehouse aisles, long
corridors), but camera images usually don't. Landfall records keyframes while the robot is well
localized, builds a visual map database in AMCL's `map` frame, and later proposes a pose from a
single camera, **verifies it with LiDAR and odometry**, and publishes `/initialpose` only when every
check passes. A wrong pose is worse than no pose, so Landfall prefers a miss to a false positive.

- Any single calibrated camera. The default backend (ORB features, bag of words) runs on a CPU;
  learned features and descriptors on ONNX Runtime are optional.
- Drop-in on existing maps (slam_toolbox, Cartographer, any map_server `.yaml`), no re-mapping.
- Every pose is checked: plausibility, the LiDAR scan against the occupancy map (also against
  every other place the scan could fit), and agreement of several frames with odometry.
- Nav2 behavior tree nodes, an example tree for each Nav2 release, launch files; a kidnap watchdog
  that notices when AMCL is confidently wrong.
- Open source (Apache-2.0) with permissively licensed dependencies only.
- ROS 2 Humble, Jazzy and Lyrical.

## Results

The kidnap benchmark, in simulation: a TurtleBot 4 in Nav2's depot world is moved to 100 random
places, the same in every row, without its localization knowing, and has to get its pose back
([`docs/benchmark.md`](docs/benchmark.md)).

| Depot world | Landfall `classical` | Landfall `learned_cpu` | AMCL's global localization |
|---|---|---|---|
| as mapped | **93 %**, 0 wrong | 78 %, 0 wrong | 6 %, **83 wrong** |
| other light | 91 %, 0 wrong | 77 %, 0 wrong | |
| nine stacks of goods the map doesn't have | 46 %, 0 wrong | 44 %, 0 wrong | 6 %, 83 wrong |
| crate stacks gone that the map still has | 68 %, 0 wrong | | 8 %, 82 wrong |

A success leaves AMCL within 0.25 m and 5° of the truth; a wrong pose is one published (or, for
AMCL, claimed) more than 0.5 m or 10° off. The rest are misses: no pose, and the robot knows it is
lost. Landfall published no wrong pose in its 800 placements (for one run of 100, the 95 % upper
bound of the wrong-pose rate is 3.6 %). Changes the map doesn't have cost successes: the LiDAR
check refuses poses where the scan no longer fits the map.

- **Time:** a median 6 s with `classical`, looking around from up to eight headings; 36 % of the
  placements in under a second. About 50 ms per camera frame on a laptop CPU, LiDAR checks
  included.
- **Kidnap watchdog**, with nothing asked: 85 % of the kidnaps noticed and undone within 90 s
  (median 39 s), no wrong pose, no needless relocalization in 43 minutes.
- **Offline**, images of one recording against another's database: 95 to 99 % of the requests
  succeed, with a median error of 1 to 2 cm
  ([`docs/relocalization.md`](docs/relocalization.md#accuracy-so-far-simulation-m4-re-run-in-m5-and-m7)).

All of this is simulation; validation on a real robot comes before v1.0.0.

## How it works

1. **Record** keyframes while the robot is well localized: camera images with their pose in the
   `map` frame, live with the recorder node or from a rosbag2.
2. **Build** the map database offline (`landfall build`): features and global descriptors of every
   keyframe, landmarks triangulated with the keyframe poses fixed, a bounded bundle adjustment. One
   SQLite file, bound to the occupancy map by its hashes.
3. **Relocalize** on request (a service, at startup, when AMCL is uncertain, or the kidnap
   watchdog): retrieve similar keyframes, match landmarks, solve the camera pose with gravity, then
   check it (sanity, LiDAR, ambiguity, odometry consistency) and hand it to AMCL.
4. **Navigate**: Nav2's behavior tree relocalizes before navigating when AMCL is uncertain, and as
   its first recovery.

[`docs/architecture.md`](docs/architecture.md) has the details and the packages.

## Quick start

Build (or use the Docker images, [`docker/README.md`](docker/README.md)):

```bash
mkdir -p ~/landfall_ws/src && cd ~/landfall_ws/src
git clone https://github.com/ali-pahlevani/Landfall.git
cd ~/landfall_ws
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The robot needs a calibrated camera (`CameraInfo`), a `LaserScan`, odometry in TF and AMCL (or
slam_toolbox localization) on its map: [`docs/sensors.md`](docs/sensors.md).

1. Record while AMCL is well localized; drive every aisle both ways:
   ```bash
   ros2 run landfall_ros recorder_node --ros-args -p autostart:=true -p bond.enabled:=false \
     -p image.topic:=/camera/image_raw -p camera_info.topic:=/camera/camera_info \
     -p output_dir:=$HOME/landfall_sessions
   ```
2. Build the database for the map AMCL uses:
   ```bash
   ros2 run landfall_ros landfall build --map site.yaml --output site.landfall.db \
     --staging $HOME/landfall_sessions/<session>
   ```
3. Start the localization (map_server, AMCL, the relocalizer) and Nav2 with relocalization in its
   behavior tree; the relocalizer finds the robot's pose at startup. This launch file includes
   nav2_bringup's, which isn't a dependency (it pulls in the simulators):
   `sudo apt install ros-$ROS_DISTRO-nav2-bringup`.
   ```bash
   ros2 launch landfall_bringup bringup.launch.xml map:=site.yaml \
     database:=site.landfall.db params_file:=nav2_params.yaml
   ```
4. Ask for a pose at any time (`dry_run: true` checks without publishing):
   ```bash
   ros2 service call /landfall_relocalizer/relocalize landfall_msgs/srv/Relocalize "{dry_run: false}"
   ```

## Documentation

| Document | |
|---|---|
| [`recording.md`](docs/recording.md) | recording keyframes, building and checking the database |
| [`relocalization.md`](docs/relocalization.md) | the relocalizer: requests, checks, triggers, monitors, offline evaluation |
| [`nav2_integration.md`](docs/nav2_integration.md) | launch files, lifecycle order and AMCL, behavior tree nodes, the example tree |
| [`sensors.md`](docs/sensors.md) | camera, odometry fusion, IMU, LiDAR, frames |
| [`tuning.md`](docs/tuning.md), [`troubleshooting.md`](docs/troubleshooting.md) | what to change, and when it doesn't work |
| [`parameters.md`](docs/parameters.md) | every parameter, generated from the code |
| [`models.md`](docs/models.md) | the learned backends: presets, models, licenses |
| [`database.md`](docs/database.md) | the database format |
| [`benchmark.md`](docs/benchmark.md) | the kidnap benchmark |
| [`field_validation.md`](docs/field_validation.md) | validating on a real robot before relying on it |
| [`PLAN.md`](docs/PLAN.md), [`TODO.md`](docs/TODO.md) | the implementation plan and decisions, open items |

## Status

Every v1.0 feature is in, tested in simulation only. The interfaces (messages, services,
parameters, database schema) may still change until the release candidate v1.0.0-rc.1, and only
compatibly from v1.0.0 on; v1.0.0 follows validation on a real robot (PLAN M8,
[`docs/field_validation.md`](docs/field_validation.md)). The learned GPU preset (CUDA, TensorRT)
and arm64 performance are experimental. [`CHANGELOG.md`](CHANGELOG.md) lists the changes.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

Apache-2.0, see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE). Third-party components are listed in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
