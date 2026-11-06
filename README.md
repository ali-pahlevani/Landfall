# Landfall

**Monocular visual relocalization for Nav2, on the 2D maps you already have.**

Robots running AMCL need an initial pose at startup and after being moved ("kidnapped"). AMCL's
global localization struggles where laser scans look alike (identical warehouse aisles, long
corridors), but camera images usually don't. Landfall records keyframes while the robot is well
localized, builds a visual map database in AMCL's `map` frame, and later proposes a pose from a
single camera, **verifies it with LiDAR and odometry**, and publishes `/initialpose` only when every
check passes. A wrong pose is worse than no pose, so Landfall prefers a miss to a false positive.

- Any single camera; CPU-only path works end to end (GPU optional).
- Drop-in on existing maps (slam_toolbox, Cartographer, any `.yaml`/`.pgm`), no re-mapping.
- Open source (Apache-2.0) with permissively licensed dependencies only.
- ROS 2 Humble, Jazzy and Lyrical.

## Status

Under development toward **v1.0.0**. Done:
- M0: skeleton, Docker, CI.
- M1: core library (features, retrieval, pose solvers, triangulation, map database, LiDAR
  verification, decision logic).
- M2: keyframe recorder and map builder, see [`docs/recording.md`](docs/recording.md).
- M3: the relocalizer node, see [`docs/relocalization.md`](docs/relocalization.md).
- M4: the safety layer: LiDAR verification of every pose, the automatic trigger, the extrinsics
  monitor and the shadow log.

Next: M5 (learned features and global descriptors on ONNX Runtime, optional).

The relocalizer has been tested in simulation only; field validation on a real robot comes last
(M8). The full plan, including interfaces, algorithms and milestones, is in
[`docs/PLAN.md`](docs/PLAN.md). Parameters are listed in [`docs/parameters.md`](docs/parameters.md).

## Building

```bash
scripts/docker_ci.sh jazzy     # or humble / lyrical: build and test in Docker, as CI does
```

See [`docker/README.md`](docker/README.md) for the images and [`docs/TODO.md`](docs/TODO.md) for
tracked open items.

## License

Apache-2.0, see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE). Third-party components are listed in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
