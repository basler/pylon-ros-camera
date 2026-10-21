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
#          For the 3D cameras (Stereo ace, Stereo mini, blaze) also download the
#          matching "pylon Supplementary Package for ..." .deb and place it next
#          to this Dockerfile, then pass it via its build arg (see below). The
#          base suite does not include the 3D camera SDKs.
#
# STEP 2 – Build examples:
#
#   # Jazzy (default on this branch):
#   docker build --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                -t pylon_ros2_camera:jazzy .   # uses REPO_BRANCH=jazzy by default
#
#   # Kilted:
#   docker build --build-arg ROS_DISTRO=kilted \
#                --build-arg REPO_BRANCH=kilted \
#                --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                -t pylon_ros2_camera:kilted .
#
#   # Jazzy with Stereo ace + Stereo mini support:
#   docker build --build-arg ROS_DISTRO=jazzy \
#                --build-arg REPO_BRANCH=jazzy \
#                --build-arg PYLON_DEB=pylon_<version>_amd64.deb \
#                --build-arg PYLON_STEREO_ACE_DEB=pylon-supplementary-package-for-stereo-ace_<version>_amd64.deb \
#                --build-arg PYLON_STEREO_MINI_DEB=pylon-supplementary-package-for-stereo-mini-sdk_<version>_amd64.deb \
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
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# Re-declare ARGs after FROM (Docker scope rule)
ARG ROS_DISTRO=jazzy
# Git branch of https://github.com/basler/pylon-ros-camera to clone.
# Defaults to the same value as ROS_DISTRO (works for humble, jazzy and kilted).
ARG REPO_BRANCH=jazzy

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
# pylon Camera Software Suite + optional 3D supplementary packages
#
# The base pylon.deb drives 2D cameras. The Stereo ace, Stereo mini and blaze
# 3D cameras each need their own supplementary .deb, which the base suite does
# not include (it provides the StereoAce/StereoMini/Blaze SDK headers the
# driver needs to build those cameras). Place every .deb next to this
# Dockerfile and pass its filename:
#   PYLON_DEB              main suite (required)
#   PYLON_STEREO_ACE_DEB   Stereo ace support (optional)
#   PYLON_STEREO_MINI_DEB  Stereo mini support (optional)
#   PYLON_BLAZE_DEB        blaze support (optional)
# Omit a supplementary arg to skip a camera you do not use.
# -----------------------------------------------------------------------------
ARG PYLON_DEB
ARG PYLON_STEREO_ACE_DEB=
ARG PYLON_STEREO_MINI_DEB=
ARG PYLON_BLAZE_DEB=

# Copy every .deb from the build context (main suite plus any supplementary ones).
COPY *.deb /tmp/debs/

# Install the main suite first so the supplementary packages find their dependency.
# Use apt-get (not plain dpkg -i) so apt pulls each package's dependencies,
# including the Stereo ace pre-dependency on desktop-file-utils.
RUN apt-get update && for pkg in "${PYLON_DEB}" "${PYLON_STEREO_ACE_DEB}" "${PYLON_STEREO_MINI_DEB}" "${PYLON_BLAZE_DEB}"; do \
        [ -n "$pkg" ] || continue; \
        echo "Installing $pkg"; \
        apt-get install -y --no-install-recommends "/tmp/debs/$pkg"; \
    done \
    && rm -rf /tmp/debs /var/lib/apt/lists/*

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
#   docker run --network=host pylon_ros2_camera:jazzy ros2 topic list
CMD ["ros2", "launch", "pylon_ros2_camera_wrapper", "pylon_ros2_camera.launch.py"]
