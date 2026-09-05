#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$workspace_dir"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS 2 is not sourced. Run: source /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

"$workspace_dir/thirdparty/build.sh"
thirdparty_prefix="$workspace_dir/install/thirdparty"

build_tests="OFF"
if [[ "${1:-}" == "test" ]]; then
  build_tests="ON"
  shift
fi

package_args=()
if [[ $# -gt 0 ]]; then
  package_args=(--packages-select "$1")
fi

colcon build \
  --symlink-install \
  "${package_args[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING="$build_tests" \
    -DCMAKE_PREFIX_PATH="$thirdparty_prefix"

echo "Build complete. Source: $workspace_dir/install/setup.bash"
