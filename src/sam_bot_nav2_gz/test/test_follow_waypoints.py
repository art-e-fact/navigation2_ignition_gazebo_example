import os
from datetime import datetime


import launch

import launch_pytest
import launch_testing
from launch_pytest.tools import process as process_tools
from launch_ros.actions import Node

import pytest
from artefacts_toolkit.chart import make_chart
from artefacts_toolkit.config import get_artefacts_param
from artefacts_toolkit.gazebo import gz
from artefacts_toolkit.rosbag import image_topics, rosbag


@pytest.fixture
def rosbag_data():
    """Create and start a rosbag recorder for the test."""
    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]

    # NOTE: Same as https://github.com/art-e-fact/artefacts-toolkit-rosbag/blob/main/artefacts_toolkit_rosbag/rosbag.py
    #   so far i couldn't find a way to get the command as string. Maybe we can add it if it's useful
    yyyymmddhhmmss = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    rosbag_filepath = os.path.join("output", "rosbag2_" + yyyymmddhhmmss)
    rosbag_cmd = (
        ["ros2", "bag", "record"]
        + topics
        + sim_topics
        + metrics
        + camera_topics
        + ["-o", rosbag_filepath, "--storage", "mcap"]
    )

    print(f"Recording to {rosbag_filepath} with command: {' '.join(rosbag_cmd)}")
    return launch.actions.ExecuteProcess(
        name="rosbag2",
        # cmd=["echo", "Recording..."],
        cmd=rosbag_cmd,
        # cmd=test_talker_node_cmd,
        shell=True,
        cached_output=True,
        output="both",
    )


@pytest.fixture(scope="module")
def navigation_stack():
    try:
        world = get_artefacts_param("launch", "world")
    except FileNotFoundError:
        world = "depot.sdf"  # Make sure this is a valid filename with extension

    # Build the ros2 launch command
    launch_cmd = [
        "ros2",
        "launch",
        "sam_bot_nav2_gz",
        "waypoint_follower_example_launch.py",
        f"world_file:={world}",
        "headless:=True",
    ]
    print(f"Starting navigation stack with command: {' '.join(launch_cmd)}")

    return launch.actions.ExecuteProcess(
        name="navigation_stack",
        cmd=launch_cmd,
        shell=True,
        cached_output=True,
        output="both",
        sigterm_timeout="600.0",
        sigkill_timeout="600.0",
    )


@pytest.fixture(scope="module")
def rosbag_recording():
    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]
    bag_recorder, rosbag_filepath = rosbag.get_bag_recorder(
        topics + sim_topics + metrics + camera_topics, use_sim_time=True
    )
    return {
        "bag_recorder": bag_recorder,
        "rosbag_filepath": rosbag_filepath,
    }


@pytest.fixture(scope="module")
def odometry_node():
    return launch.actions.ExecuteProcess(
        name="odometry_node",
        cmd=["ros2", "run", "sam_bot_nav2_gz", "test_odometry_node.py"],
        shell=True,
        cached_output=True,
        output="both",
    )


@pytest.fixture(scope="module")
def gz_bridge_node():
    """Bridge extra gazebo topics to ros2."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
        ],
        output="screen",
    )


# This function specifies the processes to be run for our test.
@launch_pytest.fixture(
    scope="module"
)  # Set the scope so the processes are not killed after each test
def launch_description(
    navigation_stack, rosbag_recording, odometry_node, gz_bridge_node
):
    return launch.LaunchDescription(
        [
            navigation_stack,
            rosbag_recording["bag_recorder"],
            odometry_node,
            gz_bridge_node,
            # Tell launch when to start the test
            # If no ReadyToTest action is added, one will be appended automatically.
            launch_pytest.actions.ReadyToTest(),
            launch_testing.util.KeepAliveProc(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_nav2_ready(navigation_stack, launch_context):
    def validate_output(output):
        assert "Nav2 is ready for use!" in output, (
            'process never printed "Nav2 is ready for use!"'
        )

    process_tools.assert_output_sync(
        launch_context, navigation_stack, validate_output, timeout=250
    )


@pytest.mark.launch(fixture=launch_description)
def test_followed_waypoints(navigation_stack, launch_context):
    """Check if the robot followed the waypoints."""

    def validate_output(output):
        assert "Goal succeeded!" in output, 'process never printed "Goal succeeded!"'

    process_tools.assert_output_sync(
        launch_context, navigation_stack, validate_output, timeout=250
    )


@pytest.mark.launch(fixture=launch_description)
def test_no_skipped_waypoints(navigation_stack, launch_context):
    """Check if the robot followed the waypoints."""

    # TODO: figure out the number of waypoints
    def validate_output(output):
        assert "Executing current waypoint: 1" in output, (
            'process never printed "Executing current waypoint: 1/1"'
        )

    process_tools.assert_output_sync(
        launch_context, navigation_stack, validate_output, timeout=250
    )


@pytest.fixture(scope="module", autouse=True)
def generate_charts(request, rosbag_recording):
    """Run after all tests have completed."""
    yield
    # This code runs after all tests complete
    print("Generating charts...")
    rosbag_filepath = rosbag_recording["rosbag_filepath"]
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
