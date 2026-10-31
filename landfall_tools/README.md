# landfall_tools

Offline Python tooling: PyTorch → ONNX model export, the model manifest with SHA-256 checksums,
the ONNX Runtime install script, benchmark plotting, and test fixture preparation.

This directory is a plain Python project, not a ROS package. `COLCON_IGNORE` keeps colcon and
rosdep away from it, so nothing at runtime can depend on it.

| Path | Content |
|---|---|
| `fixtures/depot_fixture.py` | cuts the `depot_small` test fixture out of a simulation recording (since M2; see `landfall_ros/test/fixtures/depot_small/README.md`) |

Model export, manifest and ONNX Runtime script arrive in M5, plotting in M7 (see `docs/PLAN.md`
§11 and `docs/TODO.md` T-003).
