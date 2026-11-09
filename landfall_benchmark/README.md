# landfall_benchmark

Evaluation of Landfall against a simulator's ground truth, and the kidnap benchmark.

- `evaluate_poses` (since M2): position and yaw errors of staging keyframes, database keyframes
  (recorded and after BA) and AMCL estimates against a ground-truth pose topic in a bag, with the
  nominal and the best-fitting world-to-map transform; with `--queries`, it scores
  `landfall query` results. See `docs/recording.md`.
- `evaluate_scan_check` (since M4): the LiDAR check on a recording, at every scan's true pose and at
  wrong poses around it. See `docs/relocalization.md`.
- `kidnap_runner` (M7): the kidnap benchmark in a Gazebo simulation. It moves the robot to random
  places (Gazebo's `set_pose` through ros_gz_bridge) and measures how Landfall, AMCL's global
  localization or the relocalizer's kidnap watchdog give it its pose back; one CSV row per
  placement. `scenarios/depot.yaml` holds its parameters for the TurtleBot 4 depot simulation, and
  `landfall_bringup/sim/scripts/benchmark_depot.sh` runs it there.
- `summarize_benchmark` (M7): the results tables of `kidnap_runner` CSV files, as Markdown, with
  95 % Clopper-Pearson upper bounds of the wrong-pose rate.

The protocol, the scenarios and the results are in `docs/benchmark.md`.
