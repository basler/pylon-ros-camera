# =============================================================================
# Dockerfile – pylon_ros2_camera driver
#
# Supports ROS 2 Humble (Ubuntu 22.04), Jazzy (Ubuntu 24.04) and
# Kilted Kaiju (Ubuntu 24.04).  The default distro matches the branch this
# file lives on.  Override with --build-arg.
#
# STEP 1 – Download the pylon Camera Software Suite .deb for your architecture
#          from: https://www2.baslerweb.com/en/downloads/software-downloads/
#          ("pylon Camera Software Suite – Linux – Debian Installer")
#          Place it next to this Dockerfile.
#
# STEP 2 – Build examples:
#
#   # Kilted (default on this branch):
#   docker build --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                -t pylon_ros2_camera:kilted .   # uses REPO_BRANCH=kilted by default
#
#   # Jazzy:
#   docker build --build-arg ROS_DISTRO=jazzy \
#                --build-arg REPO_BRANCH=jazzy \
#                --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                -t pylon_ros2_camera:jazzy .
#
#   # Humble:
#   docker build --build-arg ROS_DISTRO=humble \
#                --build-arg REPO_BRANCH=humble \
#                --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                -t pylon_ros2_camera:humble .
#
# STEP 3 – Run (GigE cameras require --network=host):
#   docker compose up   # see docker-compose.yaml
#
# NOTE: GigE cameras require --network=host (set in docker-compose.yaml).
#       Without it the camera CANNOT be discovered (issues #222, #236).
#
# Architecture: amd64 (x86_64).  For arm64, download the arm64 .deb and add
#   --platform linux/arm64 to the build command.
# =============================================================================

# ROS 2 distribution.  Each branch sets its own default:
#   kilted → kilted  |  jazzy → jazzy  |  humble → humble
ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base

# Re-declare ARGs after FROM (Docker scope rule)
ARG ROS_DISTRO=kilted
# Git branch of https://github.com/basler/pylon-ros-camera to clone.
# Defaults to the same value as ROS_DISTRO (works for humble, jazzy and kilted).
ARG REPO_BRANCH=kilted

ENV DEBIAN_FRONTEND=noninteractive \
    PYLON_ROOT=/opt/pylon \
    ROS_DISTRO=${ROS_DISTRO}

# -----------------------------------------------------------------------------
# System dependencies
# Pre-install the ROS2 packages that rosdep would install, so the build is
# reproducible even without a working rosdep cache.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-diagnostic-updater \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-camera-info-manager \
        ros-${ROS_DISTRO}-image-geometry \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# pylon Camera Software Suite
#
# Place the .deb downloaded from Basler's website in the same directory as
# this Dockerfile, then pass its filename via --build-arg PYLON_DEB=<name>.
# -----------------------------------------------------------------------------
ARG PYLON_DEB
COPY ${PYLON_DEB} /tmp/pylon.deb
RUN dpkg -i /tmp/pylon.deb \
    || (apt-get update && apt-get install -f -y --no-install-recommends) \
    && rm -f /tmp/pylon.deb \
    && rm -rf /var/lib/apt/lists/*

# Make pylon libraries visible to the dynamic linker
RUN echo "/opt/pylon/lib" > /etc/ld.so.conf.d/pylon.conf && ldconfig

# -----------------------------------------------------------------------------
# Clone the driver and build the ROS2 workspace
# -----------------------------------------------------------------------------
ARG WORKSPACE=/ros2_ws

WORKDIR ${WORKSPACE}/src
RUN git clone --depth 1 -b ${REPO_BRANCH} \
        https://github.com/basler/pylon-ros-camera \
        pylon_ros2_camera

WORKDIR ${WORKSPACE}

# Resolve remaining ROS2 dependencies via rosdep (best-effort)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update --rosdistro=${ROS_DISTRO} \
    && rosdep install \
           --from-paths src \
           --ignore-src \
           --rosdistro=${ROS_DISTRO} \
           -r -y \
    || true \
    && rm -rf /var/lib/apt/lists/*

# Build in Release mode
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build \
           --cmake-args -DCMAKE_BUILD_TYPE=Release \
           --event-handlers console_cohesion+

# -----------------------------------------------------------------------------
# Entrypoint and default command
# -----------------------------------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${WORKSPACE}

ENTRYPOINT ["/entrypoint.sh"]

# Default: start the camera driver.
# Override at runtime, e.g.:
#   docker run --network=host pylon_ros2_camera:kilted ros2 topic list
CMD ["ros2", "launch", "pylon_ros2_camera_wrapper", "pylon_ros2_camera.launch.py"]
