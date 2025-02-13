import unittest
import os
from ament_index_python.packages import get_package_share_directory
from launch_testing.actions import ReadyToTest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing.actions
import launch_testing.markers
import pytest
from artefacts_toolkit.rosbag import rosbag, image_topics
from artefacts_toolkit.chart import make_chart


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
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
        launch_arguments=[("run_headless", "True")],
    )

    reach_goal = Node(
        package="sam_bot_nav2_gz",
        executable="reach_goal.py",
        output="screen",
    )
 
    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]
    bag_recorder, rosbag_filepath = rosbag.get_bag_recorder(
            topics + sim_topics + metrics + camera_topics, use_sim_time=True
        )

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

    return LaunchDescription(
        [
            launch_navigation_stack,
            reach_goal,
            test_odometry_node,
            gz_bridge,
            bag_recorder,
            ReadyToTest(),
        ]
    ), { "rosbag_filepath": rosbag_filepath}


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestHelloWorldProcess(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        """Check the logs to see if the navigation task is completed"""
        # 'proc_output' is an object added automatically by the launch_testing framework.
        # It captures the outputs of the processes launched in generate_test_description()
        # Refer to the documentation for further details.
        proc_output.assertWaitFor("Goal succeeded!", timeout=800, stream="stdout")


@launch_testing.post_shutdown_test()
class TestProcOutputAfterShutdown(unittest.TestCase):
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
