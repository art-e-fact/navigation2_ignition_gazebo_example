import os
import pytest
import rclpy
from launch.substitutions import (
    LaunchConfiguration,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_pytest
from launch_pytest.tools import process as process_tools
from artefacts_toolkit.config import get_artefacts_param
import sys

sys.path.append(os.path.dirname(__file__))
from sim_state import IgnitionSimStateUtil
from sim_state.metric_value import Pose
from artefacts_toolkit_testsuite.pytest import metrics_fixture
from artefacts_toolkit_config import merge_ros_params_files
#Currently requires https://github.com/art-e-fact/artefacts-toolkit-config/pull/8


ARTEFACTS_PARAMS_FILE = os.environ.get(
    "ARTEFACTS_SCENARIO_PARAMS_FILE", "scenario_params.yaml"
)




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
            source_params_file, ARTEFACTS_PARAMS_FILE, new_params_file, rosify=True
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
            ("params_file", os.path.join(repo_root, new_params_file)),
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
    # csv exports
    robot = util.get_entity("sam_bot")
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

    goal_x, goal_y = 0.8, -0.5

    # Create waypoint pose for the navigation goal in custom_odom frame
    goal_waypoint = Pose(
        x=goal_x, y=goal_y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, frame="custom_odom"
    )

    # Calculate distance to goal using the new waypoint feature
    distance_to_goal_metric = robot.distance_to(goal_waypoint)
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


    # Cleanup
    util.stop_recording()

artefacts_metrics = metrics_fixture()


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
def test_reached_goal(reach_goal_proc, launch_context, sim, artefacts_metrics):
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
    # Check if robot reached approximately the goal area (using current position)
    current_pose = robot.pose().now()
    print(
        f"Current robot position: x={current_pose.x:.3f}, y={current_pose.y:.3f}, z={current_pose.z:.3f}"
    )

    goal_x, goal_y = 0.8, -0.5

    # Create waypoint pose for the navigation goal in custom_odom frame
    goal_waypoint = Pose(
        x=goal_x, y=goal_y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, frame="custom_odom"
    )
    # Calculate distance to goal using the new waypoint feature
    distance_to_goal_metric = robot.distance_to(goal_waypoint)
    waypoint_dist = distance_to_goal_metric.now()
    # Check if assertion should pass
    artefacts_metrics["distance_to_goal"] = waypoint_dist
    # asserts
    assert waypoint_dist < 0.5, (
        f"Robot did not reach close enough to goal, distance: {waypoint_dist:.3f}m"
    )
