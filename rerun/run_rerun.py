#!/usr/bin/env python3
"""
Simple example of a ROS node that republishes some common types to Rerun.

The solution here is mostly a toy example to show how ROS concepts can be
mapped to Rerun. Fore more information on future improved ROS support,
see the tracking issue: https://github.com/rerun-io/rerun/issues/1537

NOTE: Unlike many of the other examples, this example requires a system installation of ROS
in addition to the packages from requirements.txt.
"""

import argparse
import sys

import numpy as np
import rerun as rr
from rerun.log.text import LogLevel
import rerun_urdf

try:
    import cv_bridge
    import laser_geometry
    import rclpy
    import trimesh
    from image_geometry import PinholeCameraModel
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import PoseStamped
    from numpy.lib.recfunctions import structured_to_unstructured
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
    from rclpy.time import Duration, Time
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from sensor_msgs_py import point_cloud2
    from rcl_interfaces.msg import Log as LogMsg
    from std_msgs.msg import String
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

except ImportError as e:
    print(e)
    print(
        """
Could not import the required ROS2 packages.

Make sure you have installed ROS2 (https://docs.ros.org/en/humble/index.html)
and sourced /opt/ros/humble/setup.bash

See: README.md for more details.
"""
    )
    sys.exit(1)


class Nav2Subscriber(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        super().__init__("rr_nav2")

        # Used for subscribing to latching topics
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Allow concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to TF topics
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define a mapping for transforms
        self.path_to_frame = {
            "map": "map",
            "map/odom": "odom",
            "map/odom/base_link": "base_link",
            "map/odom/base_link/base_footprint": "base_footprint",
            "map/odom/base_link/lidar_link": "lidar_link",
            "map/odom/base_link/camera_link": "camera_link",
            "map/odom/base_link/camera_link/camera_frame": "camera_frame",
        }

        # Assorted helpers for data conversionsq
        self.model = PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()
        self.laser_proj = laser_geometry.laser_geometry.LaserProjection()

        # Log a bounding box as a visual placeholder for the map
        # # TODO(jleibs): Log the real map once [#1531](https://github.com/rerun-io/rerun/issues/1531) is merged
        rr.log_obb(
            "map/box",
            half_size=[6, 6, 2],
            position=[0, 0, 1],
            color=[255, 255, 255, 255],
            timeless=True,
        )


        # Subscriptions

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Image,
            "/sky_cam",
            self.sky_cam_callback,
            1,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Image,
            "/robot_cam",
            self.robot_cam_callback,
            1,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            CameraInfo,
            "/camera_info",
            self.camera_info_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Path,
            "/plan",
            self.plan_callback,
            10,
            callback_group=self.callback_group,
        )

        # The urdf is published as latching
        self.create_subscription(
            String,
            "/robot_description",
            self.urdf_callback,
            qos_profile=latching_qos,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            LogMsg,
            "/rosout",
            self.rosout_callback,
            1000,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10, # TODO maske sure all messages are kept
            callback_group=self.callback_group,
        )

    def log_tf_as_rigid3(self, path: str, time: Time) -> None:
        """
        Helper to look up a transform with tf and log using `log_rigid3`.

        Note: we do the lookup on the client side instead of re-logging the raw transforms until
        Rerun has support for Derived Transforms [#1533](https://github.com/rerun-io/rerun/issues/1533)
        """
        # Get the parent path
        parent_path = path.rsplit("/", 1)[0]

        # Find the corresponding frames from the mapping
        child_frame = self.path_to_frame[path]
        parent_frame = self.path_to_frame[parent_path]

        # Do the TF lookup to get transform from child (source) -> parent (target)
        try:
            tf = self.tf_buffer.lookup_transform(parent_frame, child_frame, time, timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log_rigid3(path, parent_from_child=([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        except TransformException as ex:
            print("Failed to get transform: {}".format(ex))

    def sky_cam_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        rr.log_image("sky_cam", self.cv_bridge.imgmsg_to_cv2(img))
    
    def robot_cam_callback(self, img: Image) -> None:
        """Log an `Image` with `log_image` using `cv_bridge`."""
        time = Time.from_msg(img.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        rr.log_image("map/odom/base_link/camera_link/camera_frame/img", self.cv_bridge.imgmsg_to_cv2(img))
        self.log_tf_as_rigid3("map/odom/base_link/camera_link/camera_frame", time)

    def camera_info_callback(self, info: CameraInfo) -> None:
        # (Ignition) GZ publishes all CameraInfo messages on the same topic
        #  This might be avoidable in the future: https://github.com/gazebosim/gz-sensors/issues/332#issuecomment-1472661285 
        if info.header.frame_id == "sam_bot/base_link/robot_cam":
            time = Time.from_msg(info.header.stamp)
            rr.set_time_nanos("ros_time", time.nanoseconds)

            self.model.fromCameraInfo(info)
            rr.log_pinhole(
                "map/odom/base_link/camera_link/camera_frame/img",
                child_from_parent=self.model.intrinsicMatrix(),
                width=self.model.width,
                height=self.model.height,
            )

    def odom_callback(self, odom: Odometry) -> None:
        """Update transforms when odom is updated."""
        time = Time.from_msg(odom.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Capture time-series data for the linear and angular velocities
        rr.log_scalar("odometry/vel", odom.twist.twist.linear.x)
        rr.log_scalar("odometry/ang_vel", odom.twist.twist.angular.z)

        # Update the robot pose itself via TF
        self.log_tf_as_rigid3("map/odom", time)

    def scan_callback(self, scan: LaserScan) -> None:
        """
        Log a LaserScan after transforming it to line-segments.

        Note: we do a client-side transformation of the LaserScan data into Rerun
        points / lines until Rerun has native support for LaserScan style projections:
        [#1534](https://github.com/rerun-io/rerun/issues/1534)
        """
        time = Time.from_msg(scan.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        # Project the laser scan to a collection of points
        points = self.laser_proj.projectLaser(scan)
        pts = point_cloud2.read_points(points, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts)

        # Turn every pt into a line-segment from the origin to the point.
        origin = (pts / np.linalg.norm(pts, axis=1).reshape(-1, 1)) * 0.3
        segs = np.hstack([origin, pts]).reshape(pts.shape[0] * 2, 3)

        rr.log_line_segments("map/odom/base_link/lidar_link", segs, stroke_width=0.005, color=[255, 222, 185])
        self.log_tf_as_rigid3("map/odom/base_link/lidar_link", time)
        self.log_tf_as_rigid3("map/odom/base_link", time)

    def plan_callback(self, msg: Path) -> None:
        """
        Log a Path after transforming it to a line-strip.
        """
        time = Time.from_msg(msg.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        positions = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]

        rr.log_line_strip("map", positions, stroke_width=0.05)
        self.log_tf_as_rigid3("map", time)
    

    def goal_pose_callback(self, msg: PoseStamped) -> None:
        """
        Log a PoseStamped as an downwards pointing arrow.
        """
        time = Time.from_msg(msg.header.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        length = 1.2
        goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        rr.log_arrow("map", goal + [0,0,length], [0,0,-length], width_scale=0.12, color=[252, 41, 71])
        self.log_tf_as_rigid3("map", time)

    def urdf_callback(self, urdf_msg: String) -> None:
        """Log a URDF using `log_scene` from `rerun_urdf`."""
        urdf = rerun_urdf.load_urdf_from_msg(urdf_msg)

        rerun_urdf.log_scene(scene=urdf.scene, node=urdf.base_link, path="map/odom/base_link", timeless=True)
    

    def rosout_callback(self, msg: LogMsg) -> None:
        """Log ros-log messages (from /rosout)"""
        time = Time.from_msg(msg.stamp)
        rr.set_time_nanos("ros_time", time.nanoseconds)

        level = LogLevel.TRACE
        # values taken from https://docs.ros2.org/galactic/api/rclpy/api/logging.html
        if msg.level >= 50:
            level = LogLevel.CRITICAL
        elif msg.level >= 40:
            level = LogLevel.ERROR
        elif msg.level >= 30:
            level = LogLevel.WARN
        elif msg.level >= 20:
            level = LogLevel.INFO
        elif msg.level >= 10:
            level = LogLevel.DEBUG
        name = msg.name if msg.name else "unknown"
        rr.log_text_entry(f"log/{name}", f"{msg.msg} [{msg.file}:{msg.function}@{msg.line}]", level=level)


def main() -> None:
    parser = argparse.ArgumentParser(description="Republish Navigation2 related messages to Rerun.")
    rr.script_add_args(parser)
    args, unknownargs = parser.parse_known_args()
    rerun_app_id = "nav2_viz"
    rr.script_setup(args, rerun_app_id)

    # Any remaining args go to rclpy
    rclpy.init(args=unknownargs)

    turtle_subscriber = Nav2Subscriber()

    # Use the MultiThreadedExecutor so that calls to `lookup_transform` don't block the other threads
    rclpy.spin(turtle_subscriber, executor=rclpy.executors.MultiThreadedExecutor())
    turtle_subscriber.destroy_node()
    rclpy.shutdown()
    rr.script_teardown(args)


if __name__ == "__main__":
    main()
