#!/bin/bash
# =============================================================================
# Container entrypoint
# Sources the ROS2 and workspace environments, then runs whatever command is
# passed (default: launch the camera driver).
# Works with Humble, Jazzy and Kilted – ROS_DISTRO is set by the Dockerfile.
# =============================================================================
set -e

# Source the ROS 2 base installation (distro comes from the Docker ENV)
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the built workspace
source /ros2_ws/install/setup.bash

# ROS Domain ID – default 0, can be overridden by the environment
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$@"
