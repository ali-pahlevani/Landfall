# Docker

`docker/Dockerfile` serves every supported distro through the `ROS_DISTRO` build argument
(`humble`, `jazzy`, `lyrical`).

| Stage | Contents | Typical use |
|---|---|---|
| `deps` | `ros:<distro>-ros-base` + all rosdep dependencies of the workspace | `scripts/docker_ci.sh` builds and tests against it with the repo mounted |
| `lint` | `deps` + clang-format / clang-tidy (build with `ROS_DISTRO=jazzy` for the pinned v18) | `scripts/format.sh`, `scripts/tidy.sh` |
| `build` | compiles the workspace (Release, no tests) | intermediate |
| `runtime` | base image + runtime dependencies + `/opt/landfall` install | running the nodes |

```bash
scripts/docker_ci.sh jazzy                      # build + test exactly as CI does
docker build -f docker/Dockerfile --build-arg ROS_DISTRO=jazzy --target runtime -t landfall:jazzy .
docker run --rm -it landfall:jazzy
```

`scripts/docker_ci.sh` keeps per-distro build output in `.docker_ws/<distro>/` (git-ignored) and
runs as your user, so no root-owned files end up in the checkout.

## Simulation image

`docker/Dockerfile.sim` adds Gazebo, Nav2, the TurtleBot 4 simulation and RViz to the `deps`
image (`landfall-sim:<distro>`, built on the first `scripts/sim.sh` run). `scripts/sim.sh` runs a
command in it with the GPU (NVIDIA container toolkit; without it Gazebo renders in software, which
is slow), the repository, the workspace that `scripts/docker_ci.sh` built and
`.docker_ws/sim/data/` mounted at `/data`. The simulation scripts are in
`landfall_bringup/sim/scripts/` (`docs/recording.md`, `docs/nav2_integration.md`,
`docs/benchmark.md`). With `LANDFALL_SIM_GUI=1` the container opens Gazebo's window and RViz on
your display (X11 or XWayland):

```bash
LANDFALL_SIM_GUI=1 scripts/sim.sh 'bash /ws/install/landfall_bringup/share/landfall_bringup/sim/scripts/try_depot.sh /data/depot.landfall.db'
```

The simulation image keeps the Nav2 release of the `landfall-deps` image it is built on, which
holds the Nav2 packages `landfall_nav2` builds against (`nav2_behavior_tree`, `nav2_msgs`,
`nav2_util`): Nav2's packages from two releases crash its navigator (`bt_navigator` "process has
died", exit code -11, with an older `nav2_behavior_tree` next to a newer `nav2_bt_navigator`).
After a ROS sync the apt repository has a newer Nav2 than the dependency image, and building the
simulation image stops with "Nav2 packages from more than one release". Then refresh the dependency
image (without Docker's build cache), the workspace and the simulation image, in that order:

```bash
LANDFALL_REBUILD_IMAGE=1 scripts/docker_ci.sh jazzy
LANDFALL_REBUILD_IMAGE=1 scripts/sim.sh true
```

The container runs as your user, and so does `docker exec -it <container> bash`, a shell next to
the simulation (source `/opt/ros/jazzy/setup.bash` and `/ws/install/setup.bash`). A shell of
another user (`-u root`) sees the topics but none of their messages, since DDS shared memory
belongs to the user that made it.
