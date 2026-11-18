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

The relocalizer publishes `/initialpose` with the pose's own covariance: typically 2.5-4 cm and
0.2-0.25°, longer along the view where the landmarks are far (`docs/relocalization.md`, *The
published covariance*; `output.covariance: fixed` publishes `output.std_xy` and
`output.std_yaw_deg`, 0.25 m and 5°, instead). AMCL draws its particles from it (x, y, their
correlation and yaw). AMCL from Lyrical on (Nav2 1.5) rejects a pose whose covariance has a
negative entry, a negative correlation too ("Received initialpose message is malformed.
Rejecting."), so the relocalizer publishes a cover without negative entries on every distribution:
negatively correlated pairs lose their correlation and get wider variances instead. The pose is
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
│       │   └── RetryUntilSuccessful     up to 11 turns
│       │       └── Sequence: Spin 30°, ReLocalize
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
30° and asks again, from up to twelve headings, so one of them is at most 15° off a mapped
direction. In the kidnap benchmark's depot as mapped, a request succeeded 92 % of the time within
15° of a direction that a keyframe within 1 m looked in, 66 % at 15° to 30° and 38 % at 30° to
45° (the TurtleBot's camera is 72° wide). Before PLAN V63 the tree turned by 45°, from eight
headings, so the nearest could be 22.5° off: of the same 100 placements 97 localized instead of
99, and where both did, in about the same time (`docs/benchmark.md`). A camera with a narrower
view shares less of the image with a recorded view at the same angle and needs smaller steps; a
wider one may take larger: `spin_dist` sets the step and `num_attempts` the turns (45° and 7 as
before). Each attempt lets the relocalizer take 2.5 s; where the camera, standing still, sees
nothing near a pose, the relocalizer answers after two frames (`query.early_stop_*`,
`docs/tuning.md`). In the simulation run below (run 11) such answers came after 0.4 s, or 0.7 s
right after a turn, and a 30° turn took 0.8 s: each heading that found nothing cost about 1.5 s.
From those times, twelve headings take about 17 s where the camera sees nothing near a pose at
any of them, and about 39 s where every request runs its 2.5 s.

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
| Run 9 (M6's final tree) | first attempt NO_MATCH; after one 45° turn, localized at (10.17, -6.01, 163.0°) against the true (10.15, -6.03, 162.9°); the goal succeeded 19 s later |
| Run 10 (M6's final tree) | the robot had stopped in the lane between the pallet rows, at (9.74, -3.70), which the recording never drove: no heading matched, and the goal timed out |

The earlier runs found what M6's final tree does differently: the relocalization tried one heading
in runs 2 and 3 and four, 90° apart, in runs 4 to 6, and often found no match; the controller
followed the old path and the planner saw phantom obstacles after a relocalization (run 3); and the
path was planned before AMCL's transform followed the new pose (run 8). In run 7, eight headings
found no match where the robot had stopped, as in run 10.

Run 11 (2026-11-15) used PLAN V63's tree, which turns by 30° and whose requests end early, with
the kidnap benchmark's `classical` database (M7's build of the same drive). The startup
relocalization published after 10 s, 3.2 cm and 0.3° off, and the first goal succeeded. After the
kidnap the robot drove on until the controller failed to make progress, and the first recovery's
first answer came 72 s after the kidnap. Its first three headings found nothing, each request
ending early (0.4 to 0.7 s); after three 30° turns of 0.8 s each, the fourth heading localized the
robot at (10.11, -6.00, 173.0°) against the true (10.08, -6.02, 172.8°), 5.3 s after the first
request, and the goal succeeded 19 s later.

Runs 12 to 16 (2026-11-16) repeated run 11 with the kidnap watchdog on (PLAN V64;
`RELOCALIZER_PARAMS` set to `sim/params/relocalizer_depot_watchdog.yaml`, since the simulation's
own parameters turn it off). The startup relocalization published after 11 to 18 s, 3 cm and 0.3°
off, and the first goal succeeded in every run. After the kidnap the robot drove on from the wrong
pose toward the second goal. In four runs the watchdog's relocalization published 21 to 32 s after
the kidnap, 4 to 5 cm and 0.1 to 0.3° from the truth, while AMCL was 6 to 12 m and 167° to 176°
off; each time the goal under way failed 1.0 s later. The controller's path, planned from the wrong
pose, no longer reached the robot (MPPI: "Resulting plan has 0 poses in it", `INVALID_PATH`), and
the Jazzy tree, like Nav2's own (PLAN V48), runs the controller's recovery only for the errors
`WouldAControllerRecoveryHelp` lists (`UNKNOWN`, `PATIENCE_EXCEEDED`, `FAILED_TO_MAKE_PROGRESS`,
`NO_VALID_CONTROL`; the trees handle it since PLAN V66, below). In the fifth run AMCL jumped on its own, 2.2 m and 35° from one wrong
pose to another, 19 s after the kidnap, and the goal failed the same way 2 s later, before the
watchdog's relocalization, which the end of the run cut off. In those four runs, the robot stopped
with its pose right within about half a minute of the kidnap. Without the watchdog, it drove on
blind for 72 s (run 11) to 6 minutes (M6) until the tree's recovery relocalized it, and then
reached the goal in runs 9 and 11.

Runs 17 to 22 (2026-11-16) repeated runs 12 to 16 with PLAN V66's trees: on Jazzy and Lyrical,
when the controller reports that the path no longer reaches the robot (`INVALID_PATH`), the tree
clears both costmaps and plans again from the new pose, and the goal goes on. Five runs had the
watchdog on, one had it off. With the watchdog, the robot's pose was right again 23 to 35 s after
the kidnap in every run, 3 to 10 cm and at most 0.4° from the truth, and the robot reached the second
goal in four of the five (0.18 to 0.24 m from it, AMCL 4 to 7 cm off):
- In three, the watchdog relocalized and the controller failed 1.0 s later with `INVALID_PATH`; the
  tree cleared the costmaps and planned again, and the goal went on. In one of them the plan from
  the new pose failed too, and the tree's recovery relocalized once more (at its first heading)
  before it went on.
- In one, the planner failed while the robot drove from the wrong pose, before the watchdog's
  second check, and the tree's own recovery relocalized (31 s after the kidnap, at its second
  heading).
- In one, the goal ended as reached 0.5 s before the watchdog published: AMCL's wrong pose had
  reached the goal while the robot stood 6.0 m from it. Nav2 judges a goal by the localization's
  pose; the robot then stood with its pose right, short of the goal (T-018).

Without the watchdog (run 22), the robot drove on from the wrong pose until the controller failed;
the tree's recovery first asked 86 s after the goal was sent, and its two look-arounds (24
requests) found no pose where the robot had stopped, a lane the recording never drove. The goal
failed after 8 minutes, with AMCL still 5.6 m off.

Runs 23 to 27 (2026-11-17) repeated runs 17 to 21 with PLAN V70's watchdog, which checks at once
when the scan stops fitting the map at AMCL's pose or AMCL's pose jumps, and publishes its last
check's pose when the last two checks agree. The pose was right again 9.9 to 17.0 s after the
kidnap (median 10.9 s; runs 17 to 21: 22.6 to 34.5 s, median 31.2 s), published 1.9 to 7.8 cm
from the truth, and the robot reached the second goal in all five, at most 2 cm from it. The
kidnap leaves the robot facing a wall: in every run the scan-fit trigger fired within a second or
two and its check found nothing the recording saw. The watchdog looked again once the robot had
turned toward its goal, about 5 s later, found the pose and confirmed it with its next check (in
one run the first confirmation found nothing and the next one confirmed it). In three runs the
controller failed right after the relocalization and the tree's recovery relocalized once more,
2 to 3 s later; every pose published was right.

**Limitation.** Nav2 notices a wrong pose only when the robot can't move. Until then the robot
drives on blind, and it can stop where the database holds no views of the place: the recovery can
only relocalize where the recording went, in some direction. Record every lane, in both directions,
and check the build report's coverage (docs/recording.md, "Where to drive"). A robot whose AMCL
grows uncertain relocalizes sooner, through the relocalizer's auto trigger; one whose AMCL stays
confident on the wrong pose, through its kidnap watchdog, on by default since PLAN V64
(docs/relocalization.md). In run 2, the relocalizer's background check placed the robot 9 cm from
the truth while AMCL was confidently 8 m off. Since PLAN V66 a goal under way goes on after the
watchdog's relocalization, made outside the tree: the example trees plan again when the path no
longer reaches the robot (runs 17 to 21 and 23 to 27; Lyrical's tree does the same, not run).
Since PLAN V70 the watchdog notices most kidnaps within seconds. Humble's tree runs
its recoveries after any failure (not run with the watchdog). A goal can still end on a wrong pose:
Nav2 judges it reached by the localization, which may get there before the watchdog notices
(T-018); a mission layer that checks the robot's pose again a few seconds after a goal, or after
the relocalizer published a pose (`~/status`), catches it.

### Trying it by hand

`sim/scripts/try_depot.sh` starts the same setup with Gazebo's window and RViz, for you to drive:
the depot, the TurtleBot 4 with Landfall's camera, Nav2 with the example tree, AMCL and the
relocalizer at their defaults (the startup relocalization gives AMCL its pose; the kidnap watchdog
is on). It needs a database of the depot recording (`docs/recording.md`, "The simulation
recording") and the display (`LANDFALL_SIM_GUI=1`, `docker/README.md`):

```bash
LANDFALL_SIM_GUI=1 scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/try_depot.sh /data/depot.landfall.db'
```

In RViz, "Nav2 Goal" sends the robot somewhere, and "2D Pose Estimate" gives AMCL a wrong pose,
which the kidnap watchdog notices at once and corrects. In Gazebo, select the robot and move it
with the translate or rotate tool: a kidnap, which the watchdog corrects once the camera sees a
place the recording saw. `docker exec -it <container> bash` opens a shell next to it, as your user
like the simulation (source `/opt/ros/jazzy/setup.bash` and `/ws/install/setup.bash`), for `ros2
service call /landfall_relocalizer/relocalize landfall_msgs/srv/Relocalize "{}"` or `ros2 topic
echo /landfall_relocalizer/status`. Ctrl-C ends it.

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
