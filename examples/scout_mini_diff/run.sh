#!/usr/bin/env bash
set -euo pipefail

# Source the ROS 2 workspace before running this example.
scout_share="$(ros2 pkg prefix --share neupan_ros)"

exec ros2 run neupan_ros neupan_node --ros-args \
  -p config_file:="$scout_share/config/scout_mini_diff.yaml" \
  -p dune_checkpoint:="$scout_share/models/diff_scout_mini_612x580.bin" \
  -r neupan_cmd_vel:=cmd_vel \
  "$@"
