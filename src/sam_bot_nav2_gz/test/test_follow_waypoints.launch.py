import unittest
import os
from ament_index_python.packages import get_package_share_directory
from launch_testing.actions import ReadyToTest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.markers
from launch_testing.asserts import assertInStdout
import pytest
from artefacts_toolkit.rosbag import rosbag, image_topics
from artefacts_toolkit.chart import make_chart
from artefacts_toolkit.config import get_artefacts_param
import rclpy
from gz_interfaces.srv import GetTransform
import yaml
import math


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    try:
        world = get_artefacts_param("launch", "world")
    except FileNotFoundError:
        world = "empty.world"

    run_headless = LaunchConfiguration("run_headless")
    launch_navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("sam_bot_nav2_gz"),
                    "launch",
                    "complete_navigation.launch.py"
                ),
            ]
        ),
        launch_arguments=[("run_headless", run_headless), ("world_file", world)],
    )

    follow_waypoints = Node(
        package="sam_bot_nav2_gz",
        executable="follow_waypoints.py",
        output="screen",
    )

    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]
    bag_recorder, rosbag_filepath = rosbag.get_bag_recorder(
            topics + sim_topics + metrics + camera_topics, use_sim_time=True
        )
    #rosbag_filepath = "/tmp/test.bag"
    # Gazebo ros bridge
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(
                "src",
                "sam_bot_nav2_gz",
                "test",
                 "bridge.yaml"
                )}],
        output="screen",
        )

    test_odometry_node = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(
                "src",
                "sam_bot_nav2_gz",
                "test",
                "test_odometry_node.py"
            ),
        ]
    )
    
    # Gazebo transform service node
    gazebo_transform_service = Node(
        package="sam_bot_nav2_gz",
        executable="gazebo.py",
        name="gazebo_transform_service",
        output="screen",
    )
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="run_headless",
                default_value="True",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            launch_navigation_stack,
            follow_waypoints,
            test_odometry_node,
            gz_bridge,
            gazebo_transform_service,
            bag_recorder,
            ReadyToTest(),
        ]
    ), { "rosbag_filepath": rosbag_filepath}


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFollowWaypoints(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_waypoint_checker')
        cls.get_transform_client = cls.node.create_client(GetTransform, 'get_transform')
        
        # Load waypoints
        waypoints_file = 'src/sam_bot_nav2_gz/test/waypoints.yaml'
        with open(waypoints_file, 'r') as f:
            cls.waypoints_data = yaml.safe_load(f)
        cls.waypoints = cls.waypoints_data['waypoints']
        cls.tolerance_position = 0.2  # 20cm tolerance
        cls.tolerance_orientation = 0.2  # ~11.5 degrees tolerance
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def check_waypoint_accuracy(self, robot_name: str, waypoint_index: int):
        """Check if robot is within tolerance of the specified waypoint using gazebo distance_to method"""
        # Create a client to call the gazebo service
        gazebo_client = self.node.create_client(GetTransform, 'get_transform')
        if not gazebo_client.wait_for_service(timeout_sec=5.0):
            raise AssertionError("GetTransform service not available")
            
        # Get the waypoint coordinates
        target_waypoint = self.waypoints[waypoint_index]
        
        # We would ideally call gazebo.distance_to() here, but since we're in a test
        # we'll implement the distance calculation directly using the service
        request = GetTransform.Request()
        request.name = robot_name
        
        future = gazebo_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if not future.result() or not future.result().found:
            raise AssertionError(f"Could not get transform for robot {robot_name}")
            
        tf = future.result().transform
        
        # Calculate position error
        target_pos = target_waypoint['position']
        robot_pos = tf.transform.translation
        
        pos_error = math.sqrt(
            (robot_pos.x - target_pos['x']) ** 2 +
            (robot_pos.y - target_pos['y']) ** 2 +
            (robot_pos.z - target_pos['z']) ** 2
        )
        
        # Calculate orientation error
        target_orient = target_waypoint['orientation']
        robot_orient = tf.transform.rotation
        
        dot_product = abs(
            robot_orient.x * target_orient['x'] +
            robot_orient.y * target_orient['y'] +
            robot_orient.z * target_orient['z'] +
            robot_orient.w * target_orient['w']
        )
        dot_product = min(1.0, dot_product)
        orient_error = 2.0 * math.acos(dot_product)
        
        # Check tolerances
        pos_within_tolerance = pos_error <= self.tolerance_position
        orient_within_tolerance = orient_error <= self.tolerance_orientation
        
        print(f"Waypoint {waypoint_index}: pos_err={pos_error:.3f}m, orient_err={orient_error:.3f}rad")
        
        if not pos_within_tolerance:
            raise AssertionError(f"Waypoint {waypoint_index} position error {pos_error:.3f}m exceeds tolerance {self.tolerance_position}m")
        if not orient_within_tolerance:
            raise AssertionError(f"Waypoint {waypoint_index} orientation error {orient_error:.3f}rad exceeds tolerance {self.tolerance_orientation}rad")
            
        print(f"Arrived at {waypoint_index}'th waypoint")
        return True

    def test_nav2_started(self, proc_output):
        try:
            proc_output.assertWaitFor("Nav2 active!", timeout=100, stream="stdout")
        except AssertionError as e:
            # replace the exception message with a more informative one
            raise AssertionError("Nav2 apparently failed to start") from e

    def test_followed_waypoints(self, proc_output):
        """Check the logs to see if the navigation task is completed"""
        # 'proc_output' is an object added automatically by the launch_testing framework.
        # It captures the outputs of the processes launched in generate_test_description()
        # Refer to the documentation for further details.
        try:
            proc_output.assertWaitFor("Goal succeeded!", timeout=300, stream="stdout")
        except AssertionError as e:
            # replace the exception message with a more informative one
            raise AssertionError("Failed to complete waypoint sequence") from e
    
    def test_waypoint_accuracy(self, proc_output):
        """Check that each waypoint was reached with acceptable accuracy"""
        # First ensure all waypoints were reached
        for i in range(8):
            try:
                proc_output.assertWaitFor(f"Arrived at {i}'th waypoint", timeout=10, stream="stdout")
                # Check waypoint accuracy after confirming arrival
                self.check_waypoint_accuracy('sam_bot', i)
            except AssertionError as e:
                raise AssertionError(f"Failed to reach waypoint {i} with accuracy") from e

@launch_testing.post_shutdown_test()
class TestProcOutputAfterShutdown(unittest.TestCase):

    def test_no_skipped_waypoint(self, proc_output):
        for i in range(8):
            try:
                assertInStdout(proc_output, f"Arrived at {i}'th waypoint", None)
            except AssertionError as e:
                raise AssertionError(f"Failed to reach waypoint {i}") from e

    def test_exit_code(self, rosbag_filepath):
        print(rosbag_filepath)
        make_chart(
            rosbag_filepath,
            "/odom.pose.pose.position.x",
            "/odom.pose.pose.position.y",
            field_unit="m",
            chart_name="odometry_position",
        )
        image_topics.extract_camera_image(rosbag_filepath, "/sky_cam")
        image_topics.extract_video(rosbag_filepath, "/sky_cam", "output/sky_cam.webm")
