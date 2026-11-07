# landfall_tools

Offline Python tooling: PyTorch → ONNX model export, the model manifest with SHA-256 checksums,
the ONNX Runtime install script, benchmark plotting, and test fixture preparation.

This directory is a plain Python project, not a ROS package. `COLCON_IGNORE` keeps colcon and
rosdep away from it, so nothing at runtime can depend on it.

| Path | Content |
|---|---|
| `fixtures/depot_fixture.py` | cuts the `depot_small` test fixture out of a simulation recording (since M2; see `landfall_ros/test/fixtures/depot_small/README.md`) |
| `models.yaml` | the learned models: pinned upstream files, licenses, SHA-256 of every file (M5, `docs/models.md`) |
| `fetch_models.py` | downloads and checks them (`scripts/fetch_models.sh`), or with `--upstream` the upstream files their export reads |
| `export/xfeat.py`, `export/eigenplaces.py`, `export/megaloc.py` | make the ONNX models and their parity references from the pinned upstream files (`scripts/export_models.sh` runs them all) |
| `export/parity_images.py` | the images of the parity files, from the depot fixture bag |
| `export/synthetic.py` | the synthetic models and parity files of `landfall_core/test/data/` |
| `requirements.txt` | pinned Python packages for all of the above |

The ONNX Runtime install script (for custom GPU builds) and benchmark plotting come in M7
(`docs/TODO.md` T-003).
