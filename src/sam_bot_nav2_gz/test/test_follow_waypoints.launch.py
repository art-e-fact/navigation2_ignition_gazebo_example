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

ARTEFACTS_PARAMS_FILE = os.environ.get(
    "ARTEFACTS_SCENARIO_PARAMS_FILE", "scenario_params.yaml"
)

def deep_merge_dicts(source, override):
    """Recursively merge two dictionaries, with values from `override` taking precedence over `source`"""
    for key, value in override.items():
        if isinstance(value, dict) and key in source:
            source[key] = deep_merge_dicts(source[key], value)
        else:
            source[key] = value
    return source

def merge_ros_params_files(source, override, destination):
    """Merge two ROS2 yaml parameter files into one, overriding the values in the first one with the values in `override`"""
    import yaml

    with open(source, "r") as f:
        source_params = yaml.safe_load(f)

    with open(override, "r") as f:
        override_params = yaml.safe_load(f)

    merged_params = deep_merge_dicts(source_params, override_params)
    with open(destination, "w") as f:
        yaml.dump(merged_params, f)

# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    try:
        world = get_artefacts_param("launch", "world")
    except FileNotFoundError:
        world = "empty.world"

    run_headless = LaunchConfiguration("run_headless")
    source_params_file = "src/sam_bot_nav2_gz/config/nav2_params.yaml"
    new_params_file = "all_params.yaml"
    merge_ros_params_files(source_params_file, ARTEFACTS_PARAMS_FILE, new_params_file)
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
        launch_arguments=[
            ("run_headless", run_headless),
            ("world_file", world),
            ("params_file", new_params_file),
            ],
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
            bag_recorder,
            ReadyToTest(),
        ]
    ), { "rosbag_filepath": rosbag_filepath}


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFollowWaypoints(unittest.TestCase):
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
