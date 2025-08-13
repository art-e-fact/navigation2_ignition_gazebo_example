import os
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    AndSubstitution,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_pytest
import launch_ros
from launch_pytest.tools import process as process_tools
from launch.substitutions import PathJoinSubstitution
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from artefacts_toolkit.config import get_artefacts_param
import sys
import os
sys.path.append(os.path.dirname(__file__))
from sim_state import IgnitionSimStateUtil, EntityNotFoundError
from sim_state.metric_value import Pose



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

@pytest.fixture(scope="module")
def reach_goal_proc():
    reach_goal = Node(
        package="sam_bot_nav2_gz",
        executable="reach_goal.py",
        output="screen",
        cached_output=True,
    )
    return reach_goal


@launch_pytest.fixture(scope="module")
def launch_description(reach_goal_proc, sim):
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
            #("params_file", new_params_file),
            ],
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

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")
    run_headless = LaunchConfiguration("run_headless")
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])



    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            launch_navigation_stack,
            reach_goal_proc,
            gz_bridge,
            launch_pytest.actions.ReadyToTest()
        ]
    )


@pytest.fixture(scope="module")
def sim():
    """Fixture that provides access to simulation state during tests"""
    if not rclpy.ok():
        rclpy.init()
    util = IgnitionSimStateUtil("collision_test", record_as="simulation.mcap")
    
    yield util
    
    # Cleanup
    util.stop_recording()


@pytest.mark.launch(fixture=launch_description)
def test_nav2_started(reach_goal_proc, launch_context):
    """Test that Nav2 starts successfully"""
    def validate_nav2_output(output):
        print(output)
        assert 'Nav2 active' in output, "process never printed Nav2 is ready"

    # Get the reach_goal process from the launch context
    process_tools.assert_output_sync(
            launch_context, reach_goal_proc, validate_nav2_output, timeout=40)


@pytest.mark.launch(fixture=launch_description)  
def test_reached_goal(reach_goal_proc, launch_context, sim):
    """Check that the navigation goal is reached"""
    def validate_goal_output(output):
        print(output)
        assert 'Goal succeeded!' in output, "process never printed Goal succeeded!"

    # Get the reach_goal process from the launch context
    #process_tools.wait_for_output_sync(launch_context, reach_goal_proc, validate_goal_output, timeout=15)
    process_tools.assert_output_sync(launch_context, reach_goal_proc, validate_goal_output, timeout=45)
    
    # Additional assertion using simulation state
    robot = sim.get_entity('sam_bot')

    # Create odom frame using add_transform (uses robot's pose at time 0)
    initial_robot_pose = robot.pose().at(0) #shall we use .first()
    sim.add_transform("world", "odom", initial_robot_pose)

    # Create goal waypoint in odom frame
    goal = Pose(x=0.8, y=-0.5, z=0.0, frame="odom")

    dist = robot.distance_to(goal).now()
    assert dist < 1.0, f"Robot is not close enough to goal. Distance: {dist}"
    robot.distance_to(goal).to_csv("output/test.csv")
    robot.pose().to_csv("output/pose.csv")

