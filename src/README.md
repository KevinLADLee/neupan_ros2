# ROS 2 packages

The default colcon workspace contains only native C++ runtime packages:

- `neupan_core`: ROS-independent NeuPAN C++ library.
- `neupan_ros`: rclcpp transport, preprocessing and deployment node.
- `neupan_sim`: single-process differential-drive verification simulator.

Superseded `rclpy` runtime and multi-node simulator packages are intentionally
absent from the active source tree.

Offline Python training lives in `../training` and is intentionally outside
the colcon workspace.
