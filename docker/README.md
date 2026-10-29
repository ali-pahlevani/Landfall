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
