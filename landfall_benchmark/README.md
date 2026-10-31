# landfall_benchmark

Evaluation of Landfall against a simulator's ground truth, and (from M7) the kidnap benchmark.

- `evaluate_poses` (since M2): position and yaw errors of staging keyframes, database keyframes
  (recorded and after BA) and AMCL estimates against a ground-truth pose topic in a bag, with the
  nominal and the best-fitting world-to-map transform. See `docs/recording.md`.

The kidnap benchmark runner, scenarios and metrics arrive in M7 (`docs/TODO.md` T-002).
