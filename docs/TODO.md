# TODO register

Nothing is stubbed silently. Every open item in code is written as `TODO(T-NNN): …` and must have a
row here. `scripts/check_todos.sh` enforces this in the lint workflow. Remove a row when its item
is done.

| ID | Where | What | Context | Planned |
|---|---|---|---|---|
| T-001 | `landfall_bringup` | Launch files, parameter files (incl. per-distro Nav2 overlays), RViz config | Since M2 the package holds the simulation assets (`sim/`); the robot-facing launch and parameter files come with the Nav2 integration | M6 |
| T-002 | `landfall_benchmark` | Kidnap benchmark runner, scenarios, metrics | Since M2 it holds `evaluate_poses` (ground-truth evaluation) | M7 |
| T-003 | `landfall_tools` | Model export scripts, `models.yaml` manifest, ONNX Runtime install script, benchmark plotting | Since M2 it holds the fixture tool (`fixtures/depot_fixture.py`) | M5, M7 |
| T-004 | `landfall_nav2/package.xml` | Remove the Lyrical-only `nav2_common` build dependency | Nav2 1.5.2 (Lyrical) exports `nav2_common` from `nav2_behavior_tree`'s CMake config, but `ros-lyrical-nav2-behavior-tree` doesn't depend on `ros-lyrical-nav2-common`, so `find_package(nav2_behavior_tree)` fails on a clean system (found in M0) | when fixed upstream |
| T-005 | `docs/parameters.md` | Generate the parameter reference from the nodes' declared descriptors, with a CI freshness check (PLAN §4.5) | The recorder (M2) is the only node so far; its parameters are documented by hand in `docs/recording.md` | M3 |
| T-006 | `landfall_core_build` BA | Optional extrinsics refinement (SO(3) quaternion manifold, off by default; PLAN §7.6) | No consumer yet: the M2 recordings need no extrinsics correction. Reconsider with field data | after M8 |
