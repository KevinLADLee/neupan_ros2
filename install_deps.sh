#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 is not sourced. Source /opt/ros/humble/setup.bash or /opt/ros/jazzy/setup.bash first." >&2
  exit 1
fi

case "$ROS_DISTRO" in
  humble | jazzy) ;;
  *)
    echo "Unsupported ROS_DISTRO='$ROS_DISTRO'. This workspace supports humble and jazzy." >&2
    exit 1
    ;;
esac

apt_packages=(
  build-essential
  cmake
  git
  libeigen3-dev
  libgtest-dev
  libyaml-cpp-dev
  python3-colcon-common-extensions
  "ros-$ROS_DISTRO-ament-cmake"
  "ros-$ROS_DISTRO-ament-cmake-gtest"
  "ros-$ROS_DISTRO-ament-index-python"
  "ros-$ROS_DISTRO-diagnostic-msgs"
  "ros-$ROS_DISTRO-geometry-msgs"
  "ros-$ROS_DISTRO-launch"
  "ros-$ROS_DISTRO-launch-ros"
  "ros-$ROS_DISTRO-nav-msgs"
  "ros-$ROS_DISTRO-rclcpp"
  "ros-$ROS_DISTRO-rclcpp-components"
  "ros-$ROS_DISTRO-ros2launch"
  "ros-$ROS_DISTRO-rviz2"
  "ros-$ROS_DISTRO-sensor-msgs"
  "ros-$ROS_DISTRO-std-msgs"
  "ros-$ROS_DISTRO-tf2"
  "ros-$ROS_DISTRO-tf2-ros"
  "ros-$ROS_DISTRO-visualization-msgs"
)

if [[ "$EUID" -eq 0 ]]; then
  apt-get update
  apt-get install -y "${apt_packages[@]}"
else
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required when install_deps.sh is not run as root." >&2
    exit 1
  fi
  sudo apt-get update
  sudo apt-get install -y "${apt_packages[@]}"
fi

echo "Installed NeuPAN ROS 2 dependencies for ROS 2 $ROS_DISTRO without rosdep."
