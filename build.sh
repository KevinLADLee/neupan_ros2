#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

build_tests="OFF"
package=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    test | --test)
      build_tests="ON"
      ;;
    --package)
      if [[ $# -lt 2 || -z "$2" ]]; then
        echo "--package requires a package name." >&2
        exit 2
      fi
      if [[ -n "$package" ]]; then
        echo "Only one package may be selected." >&2
        exit 2
      fi
      package="$2"
      shift
      ;;
    --package=*)
      if [[ -n "$package" ]]; then
        echo "Only one package may be selected." >&2
        exit 2
      fi
      package="${1#--package=}"
      if [[ -z "$package" ]]; then
        echo "--package requires a package name." >&2
        exit 2
      fi
      ;;
    -h | --help)
      echo "Usage: ./build.sh [test|--test] [PACKAGE|--package PACKAGE]"
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      exit 2
      ;;
    *)
      if [[ -n "$package" ]]; then
        echo "Only one package may be selected." >&2
        exit 2
      fi
      package="$1"
      ;;
  esac
  shift
done

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

missing=0
for header in /usr/include/eigen3/Eigen/Core /usr/include/yaml-cpp/yaml.h; do
  if [[ ! -f "$header" ]]; then
    echo "Missing system header: $header" >&2
    missing=1
  fi
done

if [[ ! -f thirdparty/osqp/CMakeLists.txt ||
      ! -f thirdparty/osqp-eigen/CMakeLists.txt ||
      ! -f thirdparty/qdldl/CMakeLists.txt ]]; then
  echo "Bundled solver sources are incomplete under thirdparty/." >&2
  missing=1
fi

if [[ "$missing" -ne 0 ]]; then
  echo "Install missing dependencies with: ./install_deps.sh" >&2
  exit 1
fi

package_args=()
if [[ -n "$package" ]]; then
  package_args=(--packages-up-to "$package")
fi

colcon build \
  --symlink-install \
  "${package_args[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING="$build_tests"

echo "Build complete. Source: $workspace_dir/install/setup.bash"
