# TODO register

Nothing is stubbed silently. Every open item in code is written as `TODO(T-NNN): …` and must have a
row here. `scripts/check_todos.sh` enforces this in the lint workflow. Remove a row when its item
is done.

| ID | Where | What | Context | Planned |
|---|---|---|---|---|
| T-001 | `landfall_bringup` | Launch files, parameter files (incl. per-distro Nav2 overlays), RViz config, simulation wrappers | Package created in M0 as an empty skeleton so CI covers it from the start | M6 |
| T-002 | `landfall_benchmark` | Kidnap benchmark runner, scenarios, metrics | Empty skeleton package since M0 | M7 |
| T-003 | `landfall_tools` | Model export scripts, `models.yaml` manifest, ONNX Runtime install script, benchmark plotting | Directory created in M0 with `COLCON_IGNORE` | M5, M7 |
| T-004 | `landfall_nav2/package.xml` | Remove the Lyrical-only `nav2_common` build dependency | Nav2 1.5.2 (Lyrical) exports `nav2_common` from `nav2_behavior_tree`'s CMake config, but `ros-lyrical-nav2-behavior-tree` doesn't depend on `ros-lyrical-nav2-common`, so `find_package(nav2_behavior_tree)` fails on a clean system (found in M0) | when fixed upstream |
