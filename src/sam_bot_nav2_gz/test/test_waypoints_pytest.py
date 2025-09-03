import os
import pytest
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
from sim_state.metric_value import Pose
import yaml
from datetime import datetime
from artefacts_toolkit_testsuite.pytest import metrics_fixture
from artefacts_toolkit_testsuite.nav2 import sim_fixture, assert_nav2_started, assert_nav2_completed
from artefacts_toolkit_config import merge_ros_params_files
#Currently requires https://github.com/art-e-fact/artefacts-toolkit-config/pull/8


ARTEFACTS_PARAMS_FILE = os.environ.get(
    "ARTEFACTS_SCENARIO_PARAMS_FILE", "scenario_params.yaml"
)

# used to access simulation state. The one fom nav2 includes generation of basic csvs rendered as charts
sim = sim_fixture("collision_test", "sam_bot", output_dir="output")

artefacts_metrics = metrics_fixture()

with open(os.path.join(os.path.dirname(__file__), 'waypoints.yaml'), 'r') as f:
    waypoints = yaml.safe_load(f)


@pytest.fixture(scope="module")
def follow_waypoints_proc():
    follow_waypoints = Node(
        package="sam_bot_nav2_gz",
        executable="follow_waypoints.py",
        output="screen",
        cached_output=True,
    )
    return follow_waypoints


@launch_pytest.fixture(scope="module")
def launch_description(follow_waypoints_proc, sim): #make sure sim is initialized
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
    # Go from test file to repo root: test_follow_waypoints_pytest.py -> test/ -> sam_bot_nav2_gz/ -> src/ -> repo_root/
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
            follow_waypoints_proc,
            gz_bridge,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_nav2_started(follow_waypoints_proc, launch_context):
    """Test that Nav2 starts successfully"""
    assert_nav2_started(follow_waypoints_proc, launch_context)


@pytest.mark.launch(fixture=launch_description)
@pytest.mark.parametrize("waypoint_idx", range(len(waypoints["waypoints"])))
def test_1_reached_waypoint(follow_waypoints_proc, launch_context, sim, artefacts_metrics, waypoint_idx):
    """Check that each waypoint is reached"""
    # Somehow the test does not seem to be called before the end of the full waypoints navigation, preventing the use of now()

    # Additional assertion using simulation state - wait for entity to be available
    print("Now checking entity state after goal completion...")
    robot = sim.get_entity("sam_bot")

    # Test waypoint distance functionality using the new waypoint feature
    print("Testing waypoint distance to navigation goal...")
    robot_initial_pose = robot.pose().earliest()
    sim.add_transform("world", "custom_odom", robot_initial_pose)
    wp = waypoints["waypoints"][waypoint_idx]
    # Create waypoint pose for the navigation goal in custom_odom frame
    goal_waypoint = Pose(
        x=wp["position"]["x"], y=wp["position"]["y"], z=0.0,
        qx=wp["orientation"]["x"], qy=wp["orientation"]["y"], qz=wp["orientation"]["z"], qw=wp["orientation"]["w"],
        frame="custom_odom"
    )
    sim_time = 0
    def validate_goal_output(output):
        if not output.strip():
            print("WARNING: follow_waypoints process produced no output!")
        text = f"Reached waypoint: {waypoint_idx}"
        assert text in output, f"process never printed {text}"
        # the format is f"Reached waypoint: {current_wp} @{sim_time}" we want to extract the sim_time. there could be more lines afterwards
        nonlocal sim_time
        sim_state_str = output.split(text)[1].splitlines()[0].strip().lstrip("@").strip()
        sim_time = float(sim_state_str)

    # Get the follow_waypoints process from the launch context
    print("Starting to wait for goal completion...")
    #await process_tools.assert_output(
    process_tools.assert_output_sync(
        launch_context, follow_waypoints_proc, validate_goal_output, timeout=60
    )

    current_pose = robot.pose(frame_id="custom_odom").at(sim_time)
    print(sim_time)
    print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print(
        f"Current robot position: x={current_pose.x:.3f}, y={current_pose.y:.3f}, z={current_pose.z:.3f}"
    )
    print(
        f"Goal waypoint position: x={goal_waypoint.x:.3f}, y={goal_waypoint.y:.3f}, z={goal_waypoint.z:.3f}"
    )

    # Calculate distance to goal using the new waypoint feature
    distance_to_goal_metric = robot.distance_to(goal_waypoint)
    waypoint_dist = distance_to_goal_metric.at(sim_time)
    # Check if assertion should pass
    artefacts_metrics["distance_to_goal"] = waypoint_dist
    # asserts
    assert waypoint_dist < 0.25, (
        f"Robot did not reach close enough to goal, distance: {waypoint_dist:.3f}m"
    )

@pytest.mark.launch(fixture=launch_description)
def test_2_finished_waypoints(follow_waypoints_proc, launch_context, sim, artefacts_metrics):
    """Check that each waypoint is reached"""
    assert_nav2_completed(follow_waypoints_proc, launch_context)
