#!/usr/bin/env python

"""Visualization Manager for NeuPAN ROS2 Node.

Handles all visualization-related functionality independently from main planning logic.
Provides clean separation of concerns and optional visualization with minimal overhead.

Developer: Li Chengyang <kevinladlee@gmail.com>
Date: 2025.11.15

"""

from math import sin, cos

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import MarkerArray, Marker


class VisualizationManager:
    """Manages all visualization markers for NeuPAN planner.

    Responsibilities:
    - Create and publish DUNE point cloud markers
    - Create and publish NRMP point cloud markers
    - Create and publish robot footprint markers
    - Handle enable/disable flags for each visualization type
    - Minimize data copying and lock contention

    """

    def __init__(self, node, config):
        """Initialize visualization manager.

        Args:
            node: ROS2 Node instance (for publishers, logger, clock)
            config: Configuration dictionary containing:
                - enable_visualization: bool - Master enable switch
                - enable_dune_markers: bool - Enable DUNE markers
                - enable_nrmp_markers: bool - Enable NRMP markers
                - enable_robot_marker: bool - Enable robot marker
                - map_frame: str - Map frame ID
                - marker_size: float - Marker size
                - marker_z: float - Robot marker height
                - dune_markers_topic: str - DUNE markers topic name
                - nrmp_markers_topic: str - NRMP markers topic name
                - robot_marker_topic: str - Robot marker topic name
                - state_lock: threading.Lock - Shared state lock

        """
        self.node = node

        # Enable flags
        self.enable_visualization = config['enable_visualization']
        self.enable_dune_markers = config['enable_dune_markers']
        self.enable_nrmp_markers = config['enable_nrmp_markers']
        self.enable_robot_marker = config['enable_robot_marker']

        # Visualization parameters
        self.map_frame = config['map_frame']
        self.marker_size = config['marker_size']
        self.marker_z = config['marker_z']

        # Thread safety
        self._state_lock = config['state_lock']

        # Create publishers only if visualization is enabled
        if self.enable_visualization:
            self._create_publishers(config)
            self.node.get_logger().info("Visualization enabled")
            if self.enable_dune_markers:
                self.node.get_logger().info("  - DUNE markers: enabled")
            if self.enable_nrmp_markers:
                self.node.get_logger().info("  - NRMP markers: enabled")
            if self.enable_robot_marker:
                self.node.get_logger().info("  - Robot marker: enabled")
        else:
            self.node.get_logger().info("Visualization disabled")

    def _create_publishers(self, config):
        """Create ROS2 publishers for visualization markers."""
        if self.enable_dune_markers:
            self.dune_markers_pub = self.node.create_publisher(
                MarkerArray,
                config['dune_markers_topic'],
                10
            )

        if self.enable_nrmp_markers:
            self.nrmp_markers_pub = self.node.create_publisher(
                MarkerArray,
                config['nrmp_markers_topic'],
                10
            )

        if self.enable_robot_marker:
            self.robot_marker_pub = self.node.create_publisher(
                Marker,
                config['robot_marker_topic'],
                10
            )

    def publish_visualization(self, planner, robot_state):
        """Publish all enabled visualization markers.

        Args:
            planner: NeuPAN planner object
                     (contains dune_points, nrmp_points, robot config)
            robot_state: Robot state numpy array (3, 1) [x, y, theta]

        """
        # Early exit if visualization disabled
        if not self.enable_visualization:
            return

        # Lock only to copy needed data (minimize lock time)
        with self._state_lock:
            # Only copy data for enabled markers
            if self.enable_dune_markers:
                dune_points = (
                    planner.dune_points.copy()
                    if planner.dune_points is not None else None
                )
            else:
                dune_points = None

            if self.enable_nrmp_markers:
                nrmp_points = (
                    planner.nrmp_points.copy()
                    if planner.nrmp_points is not None else None
                )
            else:
                nrmp_points = None

            if self.enable_robot_marker:
                robot_state_copy = (
                    robot_state.copy()
                    if robot_state is not None else None
                )
                # Read-only config, safe to reference
                robot_config = planner.robot
            else:
                robot_state_copy = None
                robot_config = None

        # Generate and publish markers (all outside lock)
        if self.enable_dune_markers and dune_points is not None:
            dune_markers = self._generate_dune_markers(dune_points)
            if dune_markers is not None:
                self.dune_markers_pub.publish(dune_markers)

        if self.enable_nrmp_markers and nrmp_points is not None:
            nrmp_markers = self._generate_nrmp_markers(nrmp_points)
            if nrmp_markers is not None:
                self.nrmp_markers_pub.publish(nrmp_markers)

        if self.enable_robot_marker and robot_state_copy is not None:
            robot_marker = self._generate_robot_marker(
                robot_state_copy, robot_config
            )
            if robot_marker is not None:
                self.robot_marker_pub.publish(robot_marker)

    def _generate_dune_markers(self, dune_points):
        """Generate DUNE points visualization markers.

        Args:
            dune_points: numpy array of DUNE points (2, n)

        Returns:
            MarkerArray with DUNE point markers

        """
        if dune_points is None:
            return None

        marker_array = MarkerArray()
        timestamp = self.node.get_clock().now().to_msg()

        for index, point in enumerate(dune_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp

            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0

            # Purple color for DUNE points
            marker.color.r = 160 / 255
            marker.color.g = 32 / 255
            marker.color.b = 240 / 255

            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = 0.3
            marker.pose.orientation = Quaternion()

            marker_array.markers.append(marker)

        return marker_array

    def _generate_nrmp_markers(self, nrmp_points):
        """Generate NRMP points visualization markers.

        Args:
            nrmp_points: numpy array of NRMP points (2, n)

        Returns:
            MarkerArray with NRMP point markers

        """
        if nrmp_points is None:
            return None

        marker_array = MarkerArray()
        timestamp = self.node.get_clock().now().to_msg()

        for index, point in enumerate(nrmp_points.T):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = timestamp

            marker.scale.x = self.marker_size
            marker.scale.y = self.marker_size
            marker.scale.z = self.marker_size
            marker.color.a = 1.0

            # Orange color for NRMP points
            marker.color.r = 255 / 255
            marker.color.g = 128 / 255
            marker.color.b = 0 / 255

            marker.id = index
            marker.type = Marker.CUBE
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = 0.3
            marker.pose.orientation = Quaternion()

            marker_array.markers.append(marker)

        return marker_array

    def _generate_robot_marker(self, robot_state, robot_config):
        """Generate robot footprint visualization marker.

        Args:
            robot_state: numpy array [x, y, theta] (3, 1)
            robot_config: robot configuration object
                          with shape, length, width, etc.

        Returns:
            Marker representing robot footprint

        """
        if robot_state is None or robot_config is None:
            return None

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.node.get_clock().now().to_msg()

        marker.color.a = 1.0
        # Green color for robot
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if robot_config.shape == "rectangle":
            length = robot_config.length
            width = robot_config.width
            wheelbase = robot_config.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = Marker.CUBE

            x = robot_state[0, 0]
            y = robot_state[1, 0]
            theta = robot_state[2, 0]

            # Adjust position for Ackermann kinematics
            if robot_config.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self._yaw_to_quat(theta)

        return marker

    @staticmethod
    def _yaw_to_quat(yaw):
        """Convert yaw angle to quaternion.

        Args:
            yaw: Yaw angle in radians

        Returns:
            Quaternion message

        """
        quater = Quaternion()
        quater.x = 0.0
        quater.y = 0.0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)
        return quater
