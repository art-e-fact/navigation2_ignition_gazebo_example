import os
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_pytest
from launch_pytest.tools import process as process_tools
from artefacts_toolkit.rosbag import rosbag, image_topics
from artefacts_toolkit.chart import make_chart
from artefacts_toolkit.config import get_artefacts_param
import sys
import os
sys.path.append(os.path.dirname(__file__))
from simulation_state_util import SimulationStateUtil


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

@pytest.fixture()
def reach_goal_proc():
    reach_goal = Node(
        package="sam_bot_nav2_gz",
        executable="reach_goal.py",
        output="screen",
        cached_output=True,
    )
    return reach_goal


@launch_pytest.fixture
def launch_description(reach_goal_proc):
    """Launch description fixture for pytest-based testing"""
    try:
        world = get_artefacts_param("launch", "world", default="empty.sdf")
    except FileNotFoundError:
        world = "empty.sdf"

    run_headless = LaunchConfiguration("run_headless")
    source_params_file = "src/sam_bot_nav2_gz/config/nav2_params.yaml"
    new_params_file = "all_params.yaml"
    try:
        merge_ros_params_files(source_params_file, ARTEFACTS_PARAMS_FILE, new_params_file)
    except FileNotFoundError:
        pass
    
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
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            ),
            launch_navigation_stack,
            reach_goal_proc,
            test_odometry_node,
            gz_bridge,
            #bag_recorder,
        ]
    )


@pytest.fixture(scope="function")
def simulation_state_util():
    """Fixture that provides access to simulation state during tests"""
    if not rclpy.ok():
        rclpy.init()
    util = SimulationStateUtil()
    
    # Wait for initial state
    util.wait_for_initial_state(timeout=15.0)
    
    yield util
    
    # Cleanup
    util.shutdown_util()


@pytest.mark.launch(fixture=launch_description)
def test_nav2_started(reach_goal_proc, launch_context):
    """Test that Nav2 starts successfully"""
    def validate_nav2_output(output):
        return output and 'Nav2 active!' in output

    # Get the reach_goal process from the launch context
    process_tools.wait_for_output_sync(
            launch_context, reach_goal_proc, validate_nav2_output, timeout=30)


@pytest.mark.launch(fixture=launch_description)  
def test_reached_goal(reach_goal_proc,launch_context, simulation_state_util):
    """Check that the navigation goal is reached"""
    def validate_goal_output(output):
        return output and 'Goal succeeded!' in output

    # Get the reach_goal process from the launch context
    process_tools.wait_for_output_sync(
        launch_context, reach_goal_proc, validate_goal_output, timeout=40)
    
    # Additional assertion using simulation state
    goal_coordinates = (2.0, 3.0)  # Example goal coordinates - adjust as needed
    distance = simulation_state_util.get("robot").distance_to(goal_coordinates)
    assert distance < 1.0, f"Robot is not close enough to goal at {goal_coordinates}. Distance: {distance}"


