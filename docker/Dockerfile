# syntax=docker/dockerfile:1
#
# One Dockerfile for every supported distro:
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=<humble|jazzy|lyrical> --target <stage> .
#
# Stages:
#   deps     ROS base image + every rosdep dependency of the workspace (used by scripts/docker_ci.sh)
#   lint     deps + clang-format/clang-tidy (use with ROS_DISTRO=jazzy: version 18, the pinned one)
#   build    compiles the workspace
#   runtime  ROS base image + runtime dependencies + the installed workspace

ARG ROS_DISTRO=jazzy

# Package manifests only, so the dependency layer below is rebuilt only when a package.xml changes.
FROM ros:${ROS_DISTRO}-ros-base AS manifests
COPY . /src
RUN mkdir -p /manifests \
 && cd /src \
 && find . -name package.xml -not -path './.docker_ws/*' -exec cp --parents {} /manifests/ \;

FROM ros:${ROS_DISTRO}-ros-base AS deps
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=manifests /manifests /ws/src/landfall
RUN apt-get update \
 && apt-get install -y --no-install-recommends ccache \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /ws/src --ignore-src --rosdistro "${ROS_DISTRO}" -y \
 && rm -rf /var/lib/apt/lists/* /ws/src
WORKDIR /ws

FROM deps AS lint
RUN apt-get update \
 && apt-get install -y --no-install-recommends clang-format clang-tidy \
 && rm -rf /var/lib/apt/lists/*

FROM deps AS build
ARG ROS_DISTRO
COPY . /ws/src/landfall
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ros:${ROS_DISTRO}-ros-base AS runtime
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
COPY --from=manifests /manifests /tmp/landfall_manifests
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths /tmp/landfall_manifests --ignore-src --rosdistro "${ROS_DISTRO}" \
      --dependency-types exec -y \
 && rm -rf /var/lib/apt/lists/* /tmp/landfall_manifests
COPY --from=build /ws/install /opt/landfall
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
