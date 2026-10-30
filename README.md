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
- ROS 2 Humble and Jazzy (Lyrical in progress).

## Status

Under development toward **v1.0.0**. Done: M0 (skeleton, Docker, CI) and M1 (core library: features,
retrieval, pose solvers, triangulation, map database, LiDAR verification, decision logic). Next: M2
(recorder and map builder).

Nothing is usable on a robot yet. The full plan, including interfaces, algorithms and milestones, is in
[`docs/PLAN.md`](docs/PLAN.md).

## Building

```bash
scripts/docker_ci.sh jazzy     # or humble / lyrical: build and test in Docker, as CI does
```

See [`docker/README.md`](docker/README.md) for the images and [`docs/TODO.md`](docs/TODO.md) for
tracked open items.

## License

Apache-2.0, see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE). Third-party components are listed in
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md).
