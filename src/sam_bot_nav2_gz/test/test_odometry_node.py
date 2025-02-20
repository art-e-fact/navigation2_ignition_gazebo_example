from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseArray
from std_msgs.msg import Float32
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import random

class TestOdometry(Node):
    """ ROS2 Node that subscribes to the /odom odometry topic and checks the following:
        * The odometry message is being published with no more than 1 second between message

    Additionally, it publishes:
        * the cumulative distance traveled by the robot
        * the distance between current location and start location
    """
    def __init__(self):
        super().__init__("test_odometry_node")
        self.get_logger().info("Test Odometry Node Started")
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10, callback_group=callback_group
        )
        self.ground_truth_sub = self.create_subscription(
            PoseArray, "/world/dynamic_pose/info", self.gt_callback, 10, callback_group=callback_group
        )

        self.distance_from_start_gt_pub = self.create_publisher(
            Float32, "/distance_from_start_gt", 10
        )
        self.distance_from_start_est_pub = self.create_publisher(
            Float32, "/distance_from_start_est", 10
        )
        self.odometry_error_pub = self.create_publisher(Float32, "/odometry_error", 10)
        self.start_location = Point()
        self.gt_start_location = None
        self.current_location = None
        self.last_odom_msg = None

    def report_shutdown(self):
        self.get_logger().info("Shutting down")

    def gt_callback(self, msg: PoseArray):
        #self.get_logger().info("Ground truth message received")
        if len(msg.poses) == 0:
            return
        if self.gt_start_location is None:
            self.gt_start_location = msg.poses[0].position
        self.current_location = Point(
                x=msg.poses[0].position.x - self.gt_start_location.x,
                y=msg.poses[0].position.y - self.gt_start_location.y
                )

        # calculate the ground truth distance from the start location
        distance_from_start_gt = math.sqrt(
            (self.current_location.x) ** 2
            + (self.current_location.y) ** 2
        )
        # publish the distance from the start location
        self.distance_from_start_gt_pub.publish(Float32(data=distance_from_start_gt))


    def odom_callback(self, msg: Odometry):
        """ Callback function for the odometry subscriber
        """
        #self.get_logger().info("Odometry message received")
        max_time_between_messages = 0.5
        if self.last_odom_msg:
            # check the time between messages
            if msg.header.stamp.nanosec - self.last_odom_msg.header.stamp.nanosec > max_time_between_messages * 1e9:
                self.get_logger().warn("Time between messages is too large")

            # calculate the estimated distance from the start location
            distance_from_start_est = math.sqrt(
                (msg.pose.pose.position.x) ** 2
                + (msg.pose.pose.position.y) ** 2
            )
            # publish the distance from the start location
            self.distance_from_start_est_pub.publish(Float32(data=distance_from_start_est))
            #self.get_logger().info(f"Distance from start: {distance_from_start_est}")

            # compute the position estimate error
            if self.current_location:
                odometry_error = math.sqrt(
                    (self.current_location.x - msg.pose.pose.position.x) ** 2
                    + (self.current_location.y - msg.pose.pose.position.y) ** 2
                )
                # publish the cumulative error
                self.odometry_error_pub.publish(Float32(data=odometry_error))


        self.last_odom_msg = msg


if __name__ == "__main__":
    rclpy.init()
    test_odometry_node = TestOdometry()
    # start and handle clean shutdown on KeyboardInterrupt
    
    try:
        rclpy.spin(test_odometry_node)
    except KeyboardInterrupt:
        # ideally we want to do that:
        # test_odometry_node.report_shutdown()
        # but not possible for now: https://github.com/ros2/rclpy/issues/1077
        pass
    test_odometry_node.destroy_node()
    try:
        rclpy.shutdown()
    except:
        pass
