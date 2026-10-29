# landfall_tools

Offline Python tooling: PyTorch → ONNX model export, the model manifest with SHA-256 checksums,
the ONNX Runtime install script and benchmark plotting.

This directory is a plain Python project, not a ROS package. `COLCON_IGNORE` keeps colcon and
rosdep away from it, so nothing at runtime can depend on it.

Status: skeleton only. Content arrives in milestone M5 (export, manifest, ONNX Runtime script) and
M7 (plotting); see `docs/PLAN.md` §11 and `docs/TODO.md` T-003.
