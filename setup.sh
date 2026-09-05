#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

missing=0
for header in /usr/include/eigen3/Eigen/Core /usr/include/yaml-cpp/yaml.h; do
  if [[ ! -f "$header" ]]; then
    echo "Missing system header: $header" >&2
    missing=1
  fi
done

if [[ ! -f thirdparty/osqp/CMakeLists.txt ||
      ! -f thirdparty/osqp-eigen/CMakeLists.txt ]]; then
  echo "Bundled OSQP sources are incomplete under thirdparty/." >&2
  missing=1
fi

if [[ "$missing" -ne 0 ]]; then
  exit 1
fi

echo "Native NeuPAN dependencies are available for ROS 2 $ROS_DISTRO."
echo "Bundled solver: OSQP v1.0.0 + osqp-eigen v0.11.2."
