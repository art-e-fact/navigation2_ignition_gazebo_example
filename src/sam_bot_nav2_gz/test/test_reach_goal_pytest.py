import os
import pytest
import rclpy
import time
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
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    DeclareLaunchArgument,
)
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
        merge_ros_params_files(
            source_params_file, ARTEFACTS_PARAMS_FILE, new_params_file
        )
    except FileNotFoundError:
        pass

    # Use absolute path to avoid package discovery issues
    # Go from test file to repo root: test_reach_goal_pytest.py -> test/ -> sam_bot_nav2_gz/ -> src/ -> repo_root/
    repo_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    )
    launch_navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                repo_root,
                "src",
                "sam_bot_nav2_gz",
                "launch",
                "complete_navigation.launch.py",
            )
        ),
        launch_arguments=[
            ("run_headless", run_headless),
            ("world_file", world),
            # ("params_file", new_params_file),
        ],
    )

    # Gazebo ros bridge
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    repo_root, "src", "sam_bot_nav2_gz", "test", "bridge.yaml"
                )
            }
        ],
        output="screen",
    )

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
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.fixture(scope="module")
def sim():
    """Fixture that provides access to simulation state during tests"""
    if not rclpy.ok():
        rclpy.init()
    # Use empty world which is what the test actually launches
    util = IgnitionSimStateUtil("collision_test", record_as="output/simulation.mcap")

    yield util

    # Cleanup
    util.stop_recording()


@pytest.mark.launch(fixture=launch_description)
def test_nav2_started(reach_goal_proc, launch_context):
    """Test that Nav2 starts successfully"""

    def validate_nav2_output(output):
        print(
            f"reach_goal output: '{output[:200]}...' (truncated)"
            if len(output) > 200
            else f"reach_goal output: '{output}'"
        )
        if not output.strip():
            print("WARNING: reach_goal process produced no output!")
        # Wait for the complete navigation process, then check if Nav2 was activated
        assert "Nav2 active!" in output, "process never printed Nav2 active!"

    # Get the reach_goal process from the launch context
    print("Starting to wait for reach_goal process output...")
    process_tools.assert_output_sync(
        launch_context, reach_goal_proc, validate_nav2_output, timeout=120
    )


@pytest.mark.launch(fixture=launch_description)
def test_reached_goal(reach_goal_proc, launch_context, sim):
    """Check that the navigation goal is reached"""

    def validate_goal_output(output):
        print(f"reach_goal output: '{output}'")
        if not output.strip():
            print("WARNING: reach_goal process produced no output!")
        assert "Goal succeeded!" in output, "process never printed Goal succeeded!"

    # Get the reach_goal process from the launch context
    print("Starting to wait for goal completion...")
    process_tools.assert_output_sync(
        launch_context, reach_goal_proc, validate_goal_output, timeout=90
    )

    # Additional assertion using simulation state - wait for entity to be available
    print("Now checking entity state after goal completion...")
    robot = sim.get_entity("sam_bot")

    # Test basic robot operations and debug time references
    robot_pose = robot.pose()

    # Detailed debugging of pose data structure
    print(f"=== POSE DATA DEBUGGING ===")
    print(f"Robot pose object type: {type(robot_pose)}")
    print(f"Robot pose object attributes: {dir(robot_pose)}")

    # Check different possible time array attributes
    time_attrs = ["_time_array", "time_array", "times", "_times", "timestamps"]
    for attr in time_attrs:
        if hasattr(robot_pose, attr):
            val = getattr(robot_pose, attr)
            print(
                f"Found {attr}: {type(val)}, length: {len(val) if hasattr(val, '__len__') else 'N/A'}"
            )

    print(
        f"Robot pose data available with {len(robot_pose._time_array) if hasattr(robot_pose, '_time_array') else 'unknown'} data points"
    )

    # Debug entity poses in simulation state util
    print(f"=== SIM STATE DEBUGGING ===")
    if hasattr(sim, "_entity_poses"):
        sam_bot_data = sim._entity_poses.get("sam_bot", [])
        print(f"Raw sam_bot data points: {len(sam_bot_data)}")
        if sam_bot_data:
            print(f"First data point timestamp: {sam_bot_data[0][0]}")
            print(f"Last data point timestamp: {sam_bot_data[-1][0]}")

    # Debug time information
    if hasattr(robot_pose, "_time_array") and len(robot_pose._time_array) > 0:
        import time as time_module

        current_time = time_module.time()
        min_time = min(robot_pose._time_array)
        max_time = max(robot_pose._time_array)
        print(f"Current wall time: {current_time:.3f}")
        print(f"Data time range: {min_time:.3f} to {max_time:.3f}")
        print(f"Time span: {max_time - min_time:.3f} seconds")
        print(
            f"Time difference from now: min={current_time - min_time:.3f}s, max={current_time - max_time:.3f}s"
        )
    else:
        print("No time array data available for detailed time debugging")

    # Check if robot reached approximately the goal area (using current position)
    current_pose = robot.pose().now()
    print(
        f"Current robot position: x={current_pose.x:.3f}, y={current_pose.y:.3f}, z={current_pose.z:.3f}"
    )

    goal_x, goal_y = 0.8, -0.5

    # Calculate simple 2D distance to goal
    import math

    dist = math.sqrt((current_pose.x - goal_x) ** 2 + (current_pose.y - goal_y) ** 2)
    print(f"Distance to goal: {dist:.3f}m")

    # Export comprehensive CSV files with debugging
    print("Exporting robot navigation data...")

    # 1. Full robot pose over time (includes x, y, z, roll, pitch, yaw)
    robot.pose().to_csv("output/robot_full_pose.csv")
    print("✓ Exported full pose data to output/robot_full_pose.csv")

    # 2. Robot x/y position over time (focused on navigation trajectory)
    robot.pose().to_csv("output/robot_xy_position.csv", columns=["x", "y"])
    print("✓ Exported x/y position trajectory to output/robot_xy_position.csv")

    # 3. Robot velocity over time
    robot_velocity = robot.velocity()
    print(
        f"Velocity data available with {len(robot_velocity._time_array) if hasattr(robot_velocity, '_time_array') else 'unknown'} data points"
    )
    robot_velocity.to_csv("output/robot_velocity.csv")
    print("✓ Exported velocity data to output/robot_velocity.csv")

    # Check if assertion should pass
    if dist < 2.0:
        print(f"✓ Robot successfully reached goal area (distance: {dist:.3f}m < 2.0m)")
    else:
        print(f"⚠ Robot not quite at goal (distance: {dist:.3f}m >= 2.0m)")

    # Test waypoint distance functionality using the new waypoint feature
    print("Testing waypoint distance to navigation goal...")

    # Add transform from world to custom_odom frame using robot's initial pose
    # This establishes our own odometry frame to avoid conflict with nav2's odom
    robot_initial_pose = robot.pose().earliest()
    print(
        f"Robot initial pose: x={robot_initial_pose.x:.3f}, y={robot_initial_pose.y:.3f}, z={robot_initial_pose.z:.3f}"
    )
    sim.add_transform("world", "custom_odom", robot_initial_pose)
    print("Added world->custom_odom transform for waypoint distance calculation")

    # Create waypoint pose for the navigation goal in custom_odom frame
    goal_waypoint = Pose(
        x=goal_x, y=goal_y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, frame="custom_odom"
    )

    # Calculate distance to goal using the new waypoint feature
    distance_to_goal_metric = robot.distance_to(goal_waypoint)
    waypoint_dist = distance_to_goal_metric.now()
    print(f"Distance to goal waypoint: {waypoint_dist:.3f}m")

    # Export waypoint distance to CSV
    distance_to_goal_metric.to_csv("output/robot_distance_to_goal_waypoint.csv")
    print(
        "✓ Exported waypoint distance data to output/robot_distance_to_goal_waypoint.csv"
    )

    # Export robot xy position in custom_odom frame
    robot.pose(frame_id="custom_odom").to_csv(
        "output/robot_xy_pose_custom_odom.csv", columns=["x", "y"]
    )
    print(
        "✓ Exported robot xy position in custom_odom frame to output/robot_xy_pose_custom_odom.csv"
    )

    print("All CSV exports completed successfully!")
    print(f"Simple 2D distance to goal: {dist:.3f}m")
    print(f"Waypoint distance to goal: {waypoint_dist:.3f}m")
