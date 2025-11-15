#!/usr/bin/env python

"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework, which subscribes to the laser scan and localization information, and publishes the velocity command to the robot.

Developer: Han Ruihua <hanrh@connect.hku.hk>  Li Chengyang <kevinladlee@gmail.com>
Date: 2025.04.08
"""
from math import sin, cos, atan2
import numpy as np

try:
    from neupan import neupan
    from neupan.util import get_transform
except ImportError as e:
    raise ImportError(f"Failed to import 'neupan' package: {e}. Please install neupan first.") from e

import time
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy 

class neupan_core(Node):
    def __init__(self):
        super().__init__("neupan_node")

        # get package directory
        self.pkg_dir = get_package_share_directory("neupan_ros2")

        # ROS parameters
        self.declare_parameter("neupan_config_file", "NOT SET")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "laser_link")
        self.declare_parameter("marker_size", 0.05)
        self.declare_parameter("marker_z", 1.0)
        self.declare_parameter("scan_angle_max", 3.14)
        self.declare_parameter("scan_angle_min", -3.14)
        self.declare_parameter("scan_downsample", 1)
        self.declare_parameter("scan_range_min", 0.1)
        self.declare_parameter("scan_range_max", 5.0)
        self.declare_parameter("dune_checkpoint_file", "NOT SET")
        self.declare_parameter("refresh_initial_path", False)
        self.declare_parameter("flip_angle", False)
        self.declare_parameter("include_initial_path_direction", False)

        # Topic names (configurable for flexibility)
        self.declare_parameter("cmd_vel_topic", "/neupan_cmd_vel")
        self.declare_parameter("plan_output_topic", "/neupan_plan")
        self.declare_parameter("ref_state_topic", "/neupan_ref_state")
        self.declare_parameter("initial_path_topic", "/neupan_initial_path")
        self.declare_parameter("dune_markers_topic", "/dune_point_markers")
        self.declare_parameter("robot_marker_topic", "/robot_marker")
        self.declare_parameter("nrmp_markers_topic", "/nrmp_point_markers")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("plan_input_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")

        self.planner_config_file = os.path.join(self.pkg_dir, "config", "neupan_config",  
                                                self.get_parameter("neupan_config_file").get_parameter_value().string_value)
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        self.marker_z = self.get_parameter("marker_z").get_parameter_value().double_value

        self.scan_range = np.array([
            self.get_parameter("scan_range_min").get_parameter_value().double_value,
            self.get_parameter("scan_range_max").get_parameter_value().double_value
        ])

        self.scan_angle_range = np.array([
            self.get_parameter("scan_angle_min").get_parameter_value().double_value,
            self.get_parameter("scan_angle_max").get_parameter_value().double_value
        ])


        self.scan_downsample = self.get_parameter("scan_downsample").get_parameter_value().integer_value

        dune_checkpoint_file = os.path.join(self.pkg_dir, "config", "dune_checkpoint", self.get_parameter("dune_checkpoint_file").get_parameter_value().string_value)
        self.dune_checkpoint = dune_checkpoint_file
        self.refresh_initial_path = self.get_parameter("refresh_initial_path").get_parameter_value().bool_value
        self.flip_angle = self.get_parameter("flip_angle").get_parameter_value().bool_value
        self.include_initial_path_direction = self.get_parameter("include_initial_path_direction").get_parameter_value().bool_value

        if self.refresh_initial_path:
            self.get_logger().info("Refresh initial path is enabled")

        if not self.planner_config_file:
            raise ValueError("No planner config file provided! Please set the parameter 'config_file'")

        pan = {'dune_checkpoint': self.dune_checkpoint}
        print(self.planner_config_file)
        print(pan)
        self.neupan_planner = neupan.init_from_yaml(self.planner_config_file, pan=pan)

        # Data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False

        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            10
        )
        self.plan_pub = self.create_publisher(
            Path,
            self.get_parameter("plan_output_topic").get_parameter_value().string_value,
            10
        )
        self.ref_state_pub = self.create_publisher(
            Path,
            self.get_parameter("ref_state_topic").get_parameter_value().string_value,
            10
        )
        self.ref_path_pub = self.create_publisher(
            Path,
            self.get_parameter("initial_path_topic").get_parameter_value().string_value,
            10
        )

        # For RViz visualization
        self.point_markers_pub_dune = self.create_publisher(
            MarkerArray,
            self.get_parameter("dune_markers_topic").get_parameter_value().string_value,
            10
        )
        self.robot_marker_pub = self.create_publisher(
            Marker,
            self.get_parameter("robot_marker_topic").get_parameter_value().string_value,
            10
        )
        self.point_markers_pub_nrmp = self.create_publisher(
            MarkerArray,
            self.get_parameter("nrmp_markers_topic").get_parameter_value().string_value,
            10
        )

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        scan_qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").get_parameter_value().string_value,
            self.scan_callback,
            scan_qos_profile
        )
        self.create_subscription(
            Path,
            self.get_parameter("plan_input_topic").get_parameter_value().string_value,
            self.path_callback,
            10
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").get_parameter_value().string_value,
            self.goal_callback,
            10
        )
        
        time_period = 0.02
        self.create_timer(time_period, self.run)

    def run(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )

            yaw = self.quat_to_yaw(trans.transform.rotation)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.robot_state = np.array([x, y, yaw]).reshape(3, 1)
            self.get_logger().info("robot state: {}".format(self.robot_state), once=True)

        except tf2_ros.LookupException:
            self.get_logger().info(
                f"waiting for tf for the transform from {self.base_frame} to {self.map_frame}",
                throttle_duration_sec=1.0,
            )
            return
        except tf2_ros.ConnectivityException:
            self.get_logger().warn(
                "ConnectivityException: Transform not available, waiting for connection"
            )
            return
        except tf2_ros.ExtrapolationException:
            pass

        if self.robot_state is None:
            self.get_logger().warn("waiting for robot state", throttle_duration_sec=1.0)
            return

        if (
            len(self.neupan_planner.waypoints) >= 1
            and self.neupan_planner.initial_path is None
        ):
            self.neupan_planner.set_initial_path_from_state(self.robot_state)
            print('set initial path', self.neupan_planner.initial_path)

        if self.neupan_planner.initial_path is None:
            self.get_logger().warn("waiting for neupan initial path", throttle_duration_sec=1.0)
            return

        self.ref_path_pub.publish(
            self.generate_path_msg(self.neupan_planner.initial_path)
        )

        if self.obstacle_points is None:
            self.get_logger().warn(
                "No obstacle points, only path tracking task will be performed",
                throttle_duration_sec=1.0,
            )

        action, info = self.neupan_planner(self.robot_state, self.obstacle_points)

        self.stop = info["stop"]
        self.arrive = info["arrive"]

        if info["arrive"]:
            self.get_logger().info("arrive at the target", throttle_duration_sec=0.1)

        # publish the path and velocity
        self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
        self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
        self.vel_pub.publish(self.generate_twist_msg(action))

        dune_points_makers = self.generate_dune_points_markers_msg()
        nrmp_points_makers = self.generate_nrmp_points_markers_msg()
        robot_marker = self.generate_robot_marker_msg()
        if dune_points_makers is None:
            self.get_logger().warn("No dune points to visualize")
        else:
            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())

        if nrmp_points_makers is None:
            self.get_logger().warn("No nrmp points to visualize")
        else:
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
        if robot_marker is None:
            self.get_logger().warn("No robot marker to visualize")
        else:
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

        if info["stop"]:
            self.get_logger().info(
                f"neupan stop with the min distance {self.neupan_planner.min_distance} "
                f"threshold {self.neupan_planner.collision_threshold}",
                throttle_duration_sec=0.1,
            )

    # scan callback
    def scan_callback(self, scan_msg):
        if self.robot_state is None:
            return None

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        points = []

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                i % self.scan_downsample == 0
                and distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            self.get_logger().info("No valid scan points")
            return None

        point_array = np.hstack(points)

        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.lidar_frame, rclpy.time.Time()
            )

            yaw = self.quat_to_yaw(trans.transform.rotation)
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix
            self.get_logger().info("Scan obstacle points Received", once=True)

            return self.obstacle_points

        except tf2_ros.LookupException:
            self.get_logger().info_throttle(
                1.0,
                f"waiting for tf for the transform from {self.lidar_frame} to {self.map_frame}",
            )
            return

    def path_callback(self, path):

        self.get_logger().info("target path update")

        initial_point_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                self.get_logger().info("Using the points gradient as the initial path direction", once=True)

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]

            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path:
            self.neupan_planner.set_initial_path(initial_point_list)


    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        self.goal = np.array([[x], [y], [theta]])

        print(f"set neupan goal: {[x, y, theta]}")

        self.get_logger().info("neupan goal update")
        self.get_logger().info(f"state: {self.robot_state.tolist()}")
        self.get_logger().info(f"goal: {self.goal.tolist()}")
        self.neupan_planner.update_initial_path_from_goal(self.robot_state, self.goal)
        self.neupan_planner.reset()


    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    # generate ros message
    def generate_path_msg(self, path_list):
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            # Ensure point is a 2D numpy array with shape (>=3, 1)
            point_arr = np.array(point)
            if point_arr.ndim == 1:
                # If point is 1D, reshape to (3, 1)
                point_arr = point_arr.reshape(-1, 1)
            ps.pose.position.x = float(point_arr[0, 0])
            ps.pose.position.y = float(point_arr[1, 0])
            ps.pose.orientation = self.yaw_to_quat(float(point_arr[2, 0]))

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):
        if vel is None:
            return Twist()

        speed = float(vel[0, 0])
        steer = float(vel[1, 0])

        if self.stop or self.arrive:
            return Twist()
        else:
            action = Twist()
            action.linear.x = speed
            action.angular.z = steer
            return action

    def generate_dune_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return
        else:
            points = self.neupan_planner.dune_points

            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):
        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return
        else:
            points = self.neupan_planner.nrmp_points

            for index, point in enumerate(points.T):
                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):
        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.stamp = rclpy.time.Time().to_msg()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0.0
        quater.y = 0.0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw


def main(args=None):
    rclpy.init(args=args)

    neupan_core_node = neupan_core()
    rclpy.spin(neupan_core_node)
    print("neupan start!")
    neupan_core_node.run()

    neupan_core_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
