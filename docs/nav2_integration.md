# Nav2 integration

Landfall works next to Nav2's AMCL, on the map AMCL already uses. The relocalizer gives AMCL a
pose when it has none, or a wrong one, and two behavior tree nodes let Nav2's navigator ask for one
before and during navigation:

- `landfall_relocalizer` (package `landfall_ros`, docs/relocalization.md) publishes `/initialpose`
  once a pose from vision passes every check. It runs with the localization, after AMCL.
- `ReLocalize` and `IsLocalized` (package `landfall_nav2`, plugin library
  `landfall_nav2_bt_nodes`) are behavior tree nodes for Nav2's `bt_navigator`.
- `landfall_bringup` has launch files for both, and an example tree for each Nav2 release.

Supported: ROS 2 Humble (Nav2 1.1, BehaviorTree.CPP 3), Jazzy (Nav2 1.3) and Lyrical (Nav2 1.5),
both on BehaviorTree.CPP 4.

## Launch files

| File | Starts |
|---|---|
| `localization.launch.xml` | `map_server`, `amcl` and the relocalizer under one lifecycle manager, in that order: in place of nav2_bringup's `localization_launch.py` |
| `navigation.launch.xml` | nav2_bringup's `navigation_launch.py` with Landfall's behavior tree plugin and tree |
| `bringup.launch.xml` | both |
| `relocalizer.launch.xml` | the relocalizer alone, with a lifecycle manager of its own (`use_lifecycle_manager:=false` without) |

```bash
ros2 launch landfall_bringup bringup.launch.xml map:=/path/site.yaml \
  database:=/path/site.landfall.db params_file:=/path/nav2_params.yaml
```

Arguments: `map`, `database` (built by `landfall build`, docs/recording.md), `params_file` (your
Nav2 parameters, as for nav2_bringup), `relocalizer_params_file` (default
`params/relocalizer.yaml`), `startup_relocalization` (default `true` in the localization and
bringup launch files), `namespace`, `use_sim_time`, `autostart` and `log_level`.
`navigation.launch.xml` and `bringup.launch.xml` need nav2_bringup, which `landfall_bringup` doesn't
depend on: nav2_bringup pulls in the simulators.

`params_file` holds `map_server`'s and `amcl`'s parameters, keyed by node name. With a namespace,
key them `/**/amcl` and `/**/map_server`: unlike nav2_bringup's Python launch files, XML launch
files can't rewrite the keys.

## Lifecycle order and AMCL

AMCL takes `/initialpose` once it is configured, but the relocalizer publishes only while AMCL is
active (`output.amcl_node`), so the relocalizer belongs in the localization's lifecycle manager
after `amcl`:

```yaml
lifecycle_manager_localization:
  ros__parameters:
    node_names: [map_server, amcl, landfall_relocalizer]
```

`localization.launch.xml` does this. The relocalizer keeps a bond with the lifecycle manager
(`bond.enabled`), as Nav2's servers do.

AMCL parameters that suit relocalization:

| Parameter | Value | Why |
|---|---|---|
| `set_initial_pose` | `false` | the relocalizer gives the first pose; a configured pose would be applied first |
| `always_reset_initial_pose` | `true` | after a restart, AMCL waits for a new pose instead of reusing the last one |

The relocalizer publishes `/initialpose` with the standard deviations of `output.std_xy` and
`output.std_yaw_deg` (0.25 m, 5°), from which AMCL spreads its particles and converges. The pose is
stamped with the latest odometry the relocalizer has; when AMCL's transforms are a few milliseconds
older, AMCL warns "Failed to transform initial pose in time" and sets the pose as given, which is
right: the relocalizer has already moved it to that odometry (`output.compensate_motion`).

## Starting without a pose

A robot that starts unlocalized has no `map -> odom` transform until AMCL gets a pose. Nav2 on Jazzy
and later refuses navigation goals until then ("Initial robot pose is not available"), and its
costmaps don't activate either. So the first pose has to come from the relocalizer itself:
`startup.enabled` (the launch argument `startup_relocalization`) relocalizes once after activation,
as soon as AMCL is active and camera images, camera info, the scan and TF arrive, with up to
`startup.max_attempts` attempts `startup.retry_interval` seconds apart (docs/relocalization.md).

The behavior tree's check before navigating (below) covers a robot that has a pose, but an uncertain
one.

## Behavior tree nodes

Nav2's navigator loads them from the plugin library `landfall_nav2_bt_nodes`:

| Release | `bt_navigator` parameter `plugin_lib_names` |
|---|---|
| Jazzy, Lyrical | `["landfall_nav2_bt_nodes"]`: Nav2's own plugins are loaded anyway |
| Humble | the full list of Nav2's plugins and `landfall_nav2_bt_nodes`: `landfall_bringup/params/nav2_humble.yaml` has it |

`navigation.launch.xml` gives Nav2's nodes `params/nav2_$ROS_DISTRO.yaml` and the example tree as
`default_nav_to_pose_bt_xml`, beneath your `params_file`: a key your file sets wins. So on Humble,
whose parameter files list the plugins, add `landfall_nav2_bt_nodes` to your list; on Jazzy and
Lyrical, add it if your file sets `plugin_lib_names`.

### `ReLocalize` (action)

Calls the relocalizer's `Relocalize` service and succeeds when the answer is SUCCESS.

| Port | Default | Meaning |
|---|---|---|
| `service_name` | `landfall_relocalizer/relocalize` | the relocalizer's service, relative to the navigator's namespace |
| `num_frames` | 0 | frames that must agree (0: the relocalizer's `consistency.num_frames`) |
| `timeout` | 0 | seconds the relocalizer may take (0: its `query.timeout`, 10 s) |
| `dry_run` | false | run every check but publish no pose |
| `server_timeout` | `timeout` + 10 s, or 20 s | milliseconds the node waits at most, from its start: keep it above `query.timeout` |
| `wait_for_service_timeout` | the navigator's (1 s) | milliseconds the service may take to appear |
| `status` (output) | | SUCCESS, AMBIGUOUS, NO_MATCH, REJECTED_SANITY, REJECTED_SCAN_CHECK, REJECTED_CONSISTENCY, NOT_READY or BUSY; UNAVAILABLE or TIMEOUT from the node |
| `pose` (output) | | the pose found (`PoseWithCovarianceStamped`), on SUCCESS |

Unlike Nav2's own service nodes, `ReLocalize` doesn't wait for its service when the tree is loaded.
A relocalizer that starts after the navigator, or isn't running, makes the node fail when it is
ticked, instead of making the tree fail to load and taking the navigator down with it.

The relocalizer runs one request at a time. When it answers BUSY (it is running another request,
for example its startup relocalization), `ReLocalize` asks again every 0.5 s while
`server_timeout` lasts. When the tree halts the node, the request is dropped, but the relocalizer
finishes it: a request made before then answers BUSY.

### `IsLocalized` (condition)

Succeeds while the localization's latest pose is certain enough.

| Port | Default | Meaning |
|---|---|---|
| `pose_topic` | `amcl_pose` | the localization's `PoseWithCovarianceStamped` |
| `max_std_xy` | 0.5 | metres, in the worst direction of the x/y covariance |
| `max_std_yaw_deg` | 15 | degrees |
| `max_age` | 0 | seconds; 0 accepts any age, since AMCL publishes only when its filter updates |
| `latched` | true | subscribe transient-local, as AMCL publishes `amcl_pose` |

The topic is subscribed when the tree is loaded, so AMCL's last pose is there at the first tick. The
node fails until a pose has arrived. Its thresholds equal the relocalizer's auto trigger
(`auto_trigger.max_std_xy`, `auto_trigger.max_std_yaw_deg`).

## The example tree

`share/landfall_nav2/behavior_trees/navigate_to_pose_w_relocalization.xml`, tree ID
`NavigateToPoseWithRelocalization`, is Nav2's `navigate_to_pose_w_replanning_and_recovery.xml` of
the same release with two additions:

```text
Sequence
├── Fallback "EnsureLocalized"                   before navigating
│   ├── IsLocalized
│   └── Sequence
│       ├── Fallback (looking around)
│       │   ├── ReLocalize (timeout 2.5 s)
│       │   ├── Sequence: Spin 45°, ReLocalize
│       │   ├── ...                      (8 headings in all)
│       │   └── Sequence: Spin 45°, ReLocalize
│       ├── Wait 1 s                     for AMCL's map -> odom
│       └── clear both costmaps
└── RecoveryNode "NavigateRecovery"             Nav2's tree
    ├── (navigate with replanning)
    └── (recoveries)
        └── RoundRobin
            ├── Sequence                         first recovery
            │   ├── Fallback (looking around, as above)
            │   ├── Wait 1 s
            │   ├── clear both costmaps
            │   └── ComputePathToPose            plan from the new pose
            ├── clear the costmaps
            ├── Spin
            ├── Wait
            └── BackUp
```

A database holds views of a place only in the directions the recording drove there, so a robot
that stands at an angle to them may not be recognized. When an attempt fails, the robot turns by
45° and asks again, from up to eight headings, so one of them is at most 22.5° off a mapped
direction. Each attempt lets the relocalizer take 2.5 s, so with no match at all the robot gives up
after about 35 s.

A relocalization can move the robot's pose by metres. While the pose was wrong, the scans marked
obstacles in the costmaps in the wrong places, and the path was planned from the wrong pose. So
the tree waits a second, for AMCL to publish its new `map -> odom`, then clears both costmaps, and
the recovery plans the path again before navigation resumes. Without that, the controller follows
the old path, or a path planned with the old transform, and the planner sees the phantom
obstacles: errors Nav2's recoveries don't handle, so the goal fails.

Nav2's default trees differ between releases (BehaviorTree.CPP 3 or 4, the selectors and error
codes of Jazzy, the path validation of Lyrical), so `landfall_nav2/behavior_trees/` has one tree
per release, and the package installs the one for the Nav2 it was built against. A test loads it
with all of Nav2's plugins on every release.

A robot whose localization went wrong without AMCL noticing (moved while switched off, kidnapped,
or AMCL converged on a look-alike place) navigates from the wrong pose until the controller or
planner fails. The first recovery then relocalizes from vision. On Jazzy and later, Nav2's tree
runs recoveries only for the controller and planner errors they can help with, and the example
tree keeps that. A wrong pose can also make AMCL uncertain: the relocalizer's auto trigger
(`auto_trigger.enabled`) relocalizes on its own then, without the navigator.

## Simulation run

`landfall_bringup/sim/scripts/navigate_depot.sh` runs this setup in the depot simulation (Jazzy,
TurtleBot 4, `sim/launch/depot_navigation.launch.xml`) with a database of the M2 recording's first
drive (`classical`):
1. The robot spawns at (12.0, -0.75) with AMCL unlocalized, and the relocalizer's startup
   relocalization gives AMCL its pose.
2. NavigateToPose to (3.0, -5.5).
3. Gazebo moves the robot to (21.6, -3.5) without AMCL knowing, and NavigateToPose to (12.0, -0.75)
   follows.

Nine runs (M6, runs 2 to 10, while the tree took its final form):

| | Result |
|---|---|
| Startup relocalization | every run: published 10 to 18 s after launch, at (12.01 to 12.02, -0.72, 0.3°) against the true (12.00, -0.75, 0°), 3 to 4 cm off; Nav2 activated right after |
| First goal | succeeded in every run |
| After the kidnap | the robot drove on from the wrong pose until the controller failed to make progress, and the first recovery relocalized in every run; its first answer came 1 to 6 min after the kidnap |
| Run 9 (final tree) | first attempt NO_MATCH; after one 45° turn, localized at (10.17, -6.01, 163.0°) against the true (10.15, -6.03, 162.9°); the goal succeeded 19 s later |
| Run 10 (final tree) | the robot had stopped in the lane between the pallet rows, at (9.74, -3.70), which the recording never drove: no heading matched, and the goal timed out |

The earlier runs found what the final tree does differently: the relocalization tried one heading
in runs 2 and 3 and four, 90° apart, in runs 4 to 6, and often found no match; the controller
followed the old path and the planner saw phantom obstacles after a relocalization (run 3); and the
path was planned before AMCL's transform followed the new pose (run 8). In run 7, eight headings
found no match where the robot had stopped, as in run 10.

**Limitation.** Nav2 notices a wrong pose only when the robot can't move. Until then the robot
drives on blind, and it can stop where the database holds no views of the place: the recovery can
only relocalize where the recording went, in some direction. Record every lane, in both directions
(docs/recording.md). A robot whose AMCL grows uncertain relocalizes sooner, through the
relocalizer's auto trigger; one whose AMCL stays confident on the wrong pose, through its kidnap
watchdog (`kidnap_watchdog.enabled`, docs/relocalization.md). In run 2, the relocalizer's
background check placed the robot 9 cm from the truth while AMCL was confidently 8 m off.

### On Humble

The same run on Humble uses the TurtleBot 4's own simulator on Gazebo Fortress (with its
320 x 240 camera preview) and turtlebot4_navigation's depot map, which is the Jazzy map shifted by
-8 m in x. In the Humble simulation image (`LANDFALL_SIM_DISTRO=humble scripts/sim.sh`, which runs
Cyclone DDS: with Fast DDS, the lifecycle managers lost service responses under the simulation's
load):

```bash
export LANDFALL_SIM_DISTRO=humble
scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/record_depot_humble.sh /data/humble'
scripts/sim.sh 'ros2 run landfall_ros landfall build --map /opt/ros/humble/share/turtlebot4_navigation/maps/depot.yaml \
  --output /data/depot_humble.landfall.db --staging /data/humble/sessions/depot \
  --config /ws/install/landfall_bringup/share/landfall_bringup/sim/params/build_depot_sim.yaml'
scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/navigate_depot_humble.sh /data/humble_nav /data/depot_humble.landfall.db'
```

The scripts start Gazebo's server themselves (the TurtleBot 4's launch starts its GUI), with an
explicit transport partition (`IGN_PARTITION`: the container's user has no name, and without one
the server and its clients never find each other), OGRE 2 rendering (the Create 3's description
asks for OGRE 1, which needs a display), and the robot's dock removed (the TurtleBot 4 starts on
it, in the way of every route). AMCL runs with the Jazzy simulation's tuning
(`sim/params/localization_depot_humble.yaml`): with the TurtleBot 4's own parameters it reported a
standard deviation of about 0.3 m, and the recorder took no keyframe.

One run (M7): the recording drove the route both ways and kept 439 keyframes; the database has
13,150 landmarks (reprojection RMS 0.52 px, bundle adjustment shifts below 1.1 cm). The startup
relocalization published 17 s after launch, 4.3 cm and 0.1° from the truth, and the first goal
succeeded. After the kidnap the robot drove on from the wrong pose and stopped against the south
wall, at (10.00, -7.42), where the recording never drove: the recovery's five requests, from
headings -71° to 73°, found no match (one `REJECTED_SCAN_CHECK`), and the goal aborted, as in
the Jazzy run 10 above. The recording's bag held no ground truth (rosbag2 skips hidden topics such
as `/_internal/...`; the scripts now record `/sim_ground_truth_pose`), so its keyframe poses weren't
compared with the truth.
