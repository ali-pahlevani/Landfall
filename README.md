# Landfall

[![CI](https://github.com/ali-pahlevani/Landfall/actions/workflows/ci.yml/badge.svg)](https://github.com/ali-pahlevani/Landfall/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

**Visual relocalization for Nav2, on the 2D maps you already have.** A camera proposes where the
robot is, the LiDAR and the odometry check it, and AMCL gets the pose only when every check passes.

Robots running AMCL need an initial pose at startup and after being moved ("kidnapped"). AMCL's
global localization struggles where laser scans look alike (identical warehouse aisles, long
corridors); camera images usually don't. Landfall records keyframes while the robot is well
localized, builds a visual map database in AMCL's `map` frame, and later finds the robot's pose
from a single camera. A wrong pose is worse than no pose, so Landfall prefers a miss to a false
positive.

- **Any calibrated camera, CPU only.** The default backend (ORB features, bag of words) needs
  nothing beyond OpenCV; learned features and descriptors on ONNX Runtime are optional.
- **Your existing map.** Works on any map_server `.yaml` (slam_toolbox, Cartographer, ...) in
  AMCL's frame; no re-mapping.
- **Every pose checked** before AMCL sees it: plausibility (height, tilt, inliers), the LiDAR scan
  against the occupancy map and against every other place it could fit, and the agreement of
  several frames with odometry. Each pose carries its own covariance.
- **Nav2 integration:** behavior tree nodes (`ReLocalize`, `IsLocalized`), an example tree for each
  Nav2 release that relocalizes before navigating and as its first recovery, launch files.
- **A kidnap watchdog,** on by default: it notices when AMCL is confidently wrong (the robot was
  moved, or AMCL was given a wrong pose) and corrects it, in the depot benchmark a median 6 s after
  a kidnap.
- ROS 2 **Humble, Jazzy and Lyrical**; Apache-2.0, with permissively licensed dependencies only.

## Status

**1.0.0** is the first release. Its interfaces are frozen: messages, services, parameters, launch
files, the database schema and file formats keep their meaning through 1.x
([`CHANGELOG.md`](CHANGELOG.md) lists them).

Landfall has been measured in simulation only: Nav2's TurtleBot 4 depot and warehouse, and a
simulated warehouse of the URDF Maker project. **No real robot has run it yet.** Before a robot
relies on it, validate it there: [`docs/field_validation.md`](docs/field_validation.md) is the
procedure (shadow mode, then active trials).

## Results

The kidnap benchmark, in simulation ([`docs/benchmark.md`](docs/benchmark.md)): a TurtleBot 4 with
a 640 x 480 camera is moved to 100 random places, the same 100 in every row, without its
localization knowing, and has to get its pose back. Nav2's depot (30 x 15 m), and its warehouse
(30 x 50 m of long aisles of shelving):

| World | Landfall `classical` | Landfall `learned_cpu` | AMCL's global localization |
|---|---|---|---|
| Depot as mapped | **99 %**, 0 wrong | 78 %, 0 wrong (M7) | 6 %, **83 wrong** |
| Depot in other light | 91 %, 0 wrong (M7) | 77 %, 0 wrong (M7) | |
| Depot with nine stacks of goods the map doesn't have | 54 %, 0 wrong | 44 %, 0 wrong (M7) | 6 %, 83 wrong |
| Depot with crate stacks gone that the map still has | 81 %, 0 wrong | | 8 %, 82 wrong |
| Warehouse as mapped | **100 %**, 0 wrong | | 1 %, **88 wrong** |

A success leaves AMCL within 0.25 m and 5° of the truth; a wrong pose is one published (or, for
AMCL, claimed) more than 0.5 m or 10° off. The rest are misses: no pose, and the robot knows it is
lost. Landfall published no wrong pose in any of the 800 placements above (for one run
of 100, the 95 % upper bound of the wrong-pose rate is 3.6 %). Changes the map doesn't have cost
successes, not correctness: the LiDAR check refuses poses where the scan no longer fits the map.
The depot's `classical` rows without a mark were measured on 2026-11-15 with the relocalizer whose
requests 1.0 keeps unchanged (PLAN V63), the warehouse's with 1.0 itself; rows marked M7 were
measured with M7's relocalizer (2026-11-08 and 11-09), before the fixes of PLAN V56 to V63, which
raised `classical` as mapped from 93 to 99 %.

**Kidnap watchdog**, on by default, with nothing asked of it: the robot was kidnapped (moved
without its localization knowing), or AMCL was given a wrong pose (as with RViz's 2D Pose
Estimate), and the robot turned slowly where it stood:

| Depot, 100 times each | Undone within 90 s | Time (median / 90 %) | Wrong poses | Needless relocalizations |
|---|---|---|---|---|
| Kidnap | 98 % | 6.0 s / 22.4 s | 0 | 0 in 49 minutes |
| Wrong pose given to AMCL | 99 % | 5.9 s / 25.7 s | 0 | 0 in 50 minutes |

It notices at once: the LiDAR scan stops fitting the map at AMCL's pose, or AMCL's pose jumps. The
rest of the time goes into the camera seeing a place it knows; before 1.0 (PLAN V64) the watchdog
waited for its checks every 10 s, and needed a median 34 s for a kidnap. In five navigation runs a
kidnap just before the robot's next goal was undone 10 to 17 s after it, and the robot reached
that goal every time ([`docs/nav2_integration.md`](docs/nav2_integration.md)).

- **Time:** a median 5.4 s with `classical` as mapped, looking around from up to twelve headings;
  37 % of the placements in under a second. A robot that finds no pose gives up after 39 to 45 s.
  About 50 ms per camera frame on a laptop CPU's 16 threads (0.4 s of CPU time), LiDAR checks
  included.
- **Covariance:** each pose is published with a covariance estimated for it. In the three re-run
  depot rows its standard deviations had a median of 4.1 cm and 0.27°, and 229 of the 234
  published poses lay inside their 95 % ellipse.
- **Offline**, images of one recording against another's database: 95 to 99 % of the requests
  succeed, with a median error of 1 to 2 cm
  ([`docs/relocalization.md`](docs/relocalization.md#accuracy-so-far-simulation-m4-re-run-in-m5-and-m7)).

## How it works

1. **Record** keyframes while the robot is localized: camera images with their pose in the `map`
   frame, the LiDAR scan and the odometry pose, live with the recorder node or from a rosbag2.
2. **Build** the map database offline (`landfall build`): every keyframe's scan matched to the map,
   the keyframes tied by odometry, features and global descriptors, landmarks triangulated with
   those poses, a bounded bundle adjustment. One SQLite file, bound to the occupancy map by its
   hashes, with a build report that shows where the recording didn't look.
3. **Relocalize** on request (a service, at startup, when AMCL is uncertain, or the kidnap
   watchdog): retrieve similar keyframes, match landmarks, solve the camera pose with gravity, then
   check it (sanity, LiDAR, ambiguity, odometry consistency) and hand it to AMCL with its
   covariance.
4. **Navigate:** Nav2's behavior tree relocalizes before navigating when AMCL is uncertain, and as
   its first recovery; it plans again when a relocalization from outside it moved the robot's pose.

[`docs/architecture.md`](docs/architecture.md) has the details and the packages.

## Requirements

- **Robot:** a calibrated camera publishing `sensor_msgs/Image` (raw or compressed) and
  `CameraInfo`; a 2D LiDAR (`LaserScan`); odometry in TF (`odom → base`); AMCL (or slam_toolbox's
  localization) on a 2D occupancy map; the camera's mount in the URDF. An IMU is optional (it gives
  gravity on ramps). [`docs/sensors.md`](docs/sensors.md) covers each one.
- **Computer:** Ubuntu 22.04 (Humble), 24.04 (Jazzy) or 26.04 (Lyrical), x86_64. No GPU needed.
  arm64 builds and passes the tests in CI, but hasn't been timed.

## Installation

From source:

```bash
mkdir -p ~/landfall_ws/src && cd ~/landfall_ws/src
git clone --branch v1.0.0 https://github.com/ali-pahlevani/Landfall.git
cd ~/landfall_ws
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

rosdep brings ONNX Runtime's CPU build (`onnxruntime_vendor`), so the learned backends are built
too; `-DLANDFALL_WITH_ONNXRUNTIME=OFF` leaves them out. Their models: `scripts/fetch_models.sh`
([`docs/models.md`](docs/models.md)). Docker images for every distro: [`docker/README.md`](docker/README.md).

## Quick start on a robot

1. **Record** while AMCL is localized. Drive every aisle both ways, open floor in lanes at most 2 m
   apart, and turn on the spot at aisle ends and junctions
   ([`docs/recording.md`](docs/recording.md), "Where to drive"):
   ```bash
   ros2 run landfall_ros recorder_node --ros-args -p autostart:=true -p bond.enabled:=false \
     -p image.topic:=/camera/image_raw -p camera_info.topic:=/camera/camera_info \
     -p scan.topic:=/scan -p output_dir:=$HOME/landfall_sessions
   ```
2. **Build** the database for the map AMCL uses, and read its report (coverage, landmarks,
   keyframe-pose checks):
   ```bash
   ros2 run landfall_ros landfall build --map site.yaml --output site.landfall.db \
     --staging $HOME/landfall_sessions/<session> --report-dir site_report
   ```
3. **Localize and navigate:** map_server, AMCL and the relocalizer, plus Nav2 with relocalization
   in its behavior tree. The relocalizer finds the robot's pose at startup. This launch file includes
   nav2_bringup's (`sudo apt install ros-$ROS_DISTRO-nav2-bringup`):
   ```bash
   ros2 launch landfall_bringup bringup.launch.xml map:=site.yaml \
     database:=site.landfall.db params_file:=nav2_params.yaml
   ```
   Without Nav2: `localization.launch.xml`, same arguments
   ([`docs/nav2_integration.md`](docs/nav2_integration.md)).
4. **Ask** for a pose at any time (`dry_run: true` checks without publishing), and watch the result:
   ```bash
   ros2 service call /landfall_relocalizer/relocalize landfall_msgs/srv/Relocalize "{dry_run: false}"
   ros2 topic echo /landfall_relocalizer/status
   ```
5. **Validate** on the robot before relying on it: [`docs/field_validation.md`](docs/field_validation.md).

The relocalizer's parameters (`params/relocalizer.yaml` is a starting point) are in
[`docs/parameters.md`](docs/parameters.md); [`docs/tuning.md`](docs/tuning.md) and
[`docs/troubleshooting.md`](docs/troubleshooting.md) say what to change.

## Try it in simulation

Nav2's TurtleBot 4 depot, with Landfall's 640 x 480 camera, runs in Docker (GPU rendering with the
NVIDIA container toolkit; without it, software rendering is slow). From a clone of the repository:

```bash
scripts/docker_ci.sh jazzy     # builds and tests the workspace into .docker_ws/jazzy
# Record the depot (Nav2 drives a route both ways, about 9 minutes), then build its database:
scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/record_depot.sh /data/depot_run1'
scripts/sim.sh 'ros2 run landfall_ros landfall build --map /opt/ros/jazzy/share/nav2_bringup/maps/depot.yaml \
  --bag /data/depot_run1/bag --config /ws/install/landfall_bringup/share/landfall_bringup/sim/params/build_depot_sim.yaml \
  --output /data/depot.landfall.db'
# Drive it yourself, with Gazebo's window and RViz:
LANDFALL_SIM_GUI=1 scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/try_depot.sh /data/depot.landfall.db'
```

In RViz, "Nav2 Goal" sends the robot somewhere and "2D Pose Estimate" gives AMCL a wrong pose; in
Gazebo, move the robot with the translate tool to kidnap it, and watch the kidnap watchdog put it
back. Files land in `.docker_ws/sim/data/`. The scripted runs (a navigation run with a kidnap, the
kidnap benchmark) are in [`docs/nav2_integration.md`](docs/nav2_integration.md) and
[`docs/benchmark.md`](docs/benchmark.md).

## Limitations

- **Simulation only** so far (above).
- **Only where the recording looked.** A place the recording never saw, in no direction, can't be
  recognized; the example tree looks around in 30° steps, and the build report shows the gaps.
- **Changes the map doesn't have** cost successes, not correctness: the LiDAR check refuses poses
  where the scan no longer fits the map (unmapped goods took the depot from 99 to 54 %; a site with
  goods off the map tunes the check, `docs/tuning.md`).
- **Look-alike places** that the LiDAR can't tell apart either end as misses (`AMBIGUOUS`).
- **A goal can end on a wrong pose:** Nav2 judges a goal reached by the localization, which may get
  there before the kidnap watchdog notices a kidnap (6 s in the median, 22 s or more for one in
  ten, while the camera finds no place it knows).
- **The kidnap watchdog costs CPU:** about 8 % of one core of a laptop CPU in the depot
  simulation; its `period` trades that for how soon it notices what its triggers miss.
- One monocular camera per database; 2D maps on one floor; ramps aren't mapped. The GPU backends
  (CUDA, TensorRT) are experimental.

## Documentation

| Document | |
|---|---|
| [`recording.md`](docs/recording.md) | recording keyframes, building and checking the database |
| [`relocalization.md`](docs/relocalization.md) | the relocalizer: requests, checks, triggers, monitors, the kidnap watchdog, offline evaluation |
| [`nav2_integration.md`](docs/nav2_integration.md) | launch files, lifecycle order and AMCL, behavior tree nodes, the example tree, the simulation |
| [`sensors.md`](docs/sensors.md) | camera, odometry fusion, IMU, LiDAR, frames |
| [`tuning.md`](docs/tuning.md), [`troubleshooting.md`](docs/troubleshooting.md) | what to change, and when it doesn't work |
| [`parameters.md`](docs/parameters.md) | every parameter, generated from the code |
| [`field_validation.md`](docs/field_validation.md) | validating on your robot before relying on it |
| [`models.md`](docs/models.md) | the learned backends: presets, models, licenses |
| [`database.md`](docs/database.md) | the database format |
| [`benchmark.md`](docs/benchmark.md) | the kidnap benchmark |
| [`architecture.md`](docs/architecture.md) | packages and data flow |
| [`docker/README.md`](docker/README.md) | Docker images, the simulation image |
| [`PLAN.md`](docs/PLAN.md), [`TODO.md`](docs/TODO.md) | the implementation plan and its decisions, open items |

## Versioning

Semantic versioning. Within 1.x, the frozen interfaces (CHANGELOG) stay compatible; a database
schema change comes with `landfall migrate`. [`CHANGELOG.md`](CHANGELOG.md) lists the changes.

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## License

Apache-2.0, see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE). Third-party components, the example
trees derived from Nav2's and the simulation's assets are listed in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
