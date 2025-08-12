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

    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="sam_bot_nav2_gz"
    ).find("sam_bot_nav2_gz")
    default_model_path = os.path.join(
        pkg_share, "src/description/sam_bot_description.urdf"
    )
    run_headless = LaunchConfiguration("run_headless")
    world_file_name = LaunchConfiguration("world_file")
    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    #gz_models_path = os.path.join(pkg_share, "models")
    world_path = PathJoinSubstitution([pkg_share, "world", world_file_name])
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "sam_bot",
            "-topic",
            "robot_description",
            "-z",
            "1.0",
            "-x",
            "-2.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )


    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="world_file",
                default_value="empty.sdf",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            #DeclareLaunchArgument(
            #    name="run_headless",
            #    default_value="False",
            #    description="Start GZ in headless mode and don't start RViz (overrides use_rviz)",
            #),
            #launch_navigation_stack",
            *gazebo,
            spawn_entity,
            #reach_goal_proc,
            gz_bridge,
            launch_pytest.actions.ReadyToTest()
        ]
    )


@pytest.fixture(scope="function")
def sim():
    """Fixture that provides access to simulation state during tests"""
    if not rclpy.ok():
        rclpy.init()
    util = IgnitionSimStateUtil("collision_test", record_as="simulation.mcap")
    
    yield util
    
    # Cleanup
    util.stop_recording()


#@pytest.mark.launch(fixture=launch_description)
#def test_nav2_started(reach_goal_proc, launch_context):
#    """Test that Nav2 starts successfully"""
#    def validate_nav2_output(output):
#        return output and 'Nav2 active!' in output
#
#    # Get the reach_goal process from the launch context
#    process_tools.wait_for_output_sync(
#            launch_context, reach_goal_proc, validate_nav2_output, timeout=30)
#

@pytest.mark.launch(fixture=launch_description)  
def test_reached_goal(reach_goal_proc,launch_context, sim):
    """Check that the navigation goal is reached"""
    def validate_goal_output(output):
        return output and 'Goal succeeded!' in output

    # Get the reach_goal process from the launch context
    process_tools.wait_for_output_sync(launch_context, reach_goal_proc, validate_goal_output, timeout=5)
    
    # Additional assertion using simulation state
    goal_coordinates = (2.0, 3.0)  # Example goal coordinates - adjust as needed
    entity = sim.get_entity('ros_symbol')
    cam = sim.get_entity('sky_cam/camera_link')
    dist = entity.distance_to(cam).now()
    assert dist > 1.0, f"Robot is not close enough to camera at {cam.pose().now().x}. Distance: {dist}"
    assert cam.pose().now().z > 1.0
    entity.distance_to(cam).to_csv("output/test.csv")

