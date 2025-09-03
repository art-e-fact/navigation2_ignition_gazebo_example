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
from artefacts_toolkit_testsuite.pytest import metrics_fixture
from artefacts_toolkit_testsuite.nav2 import sim_fixture, assert_nav2_started, assert_nav2_completed, assert_close_to_waypoint
from artefacts_toolkit_config import merge_ros_params_files
#Currently requires https://github.com/art-e-fact/artefacts-toolkit-config/pull/8


ARTEFACTS_PARAMS_FILE = os.environ.get(
    "ARTEFACTS_SCENARIO_PARAMS_FILE", "scenario_params.yaml"
)


# used to access simulation state. The one fom nav2 includes generation of basic csvs rendered as charts
sim = sim_fixture("collision_test", "sam_bot", output_dir="output")

artefacts_metrics = metrics_fixture()

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
def launch_description(reach_goal_proc, sim): #make sure sim is initialized
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



@pytest.mark.launch(fixture=launch_description)
def test_nav2_started(reach_goal_proc, launch_context):
    """Test that Nav2 starts successfully"""
    assert_nav2_started(reach_goal_proc, launch_context)


@pytest.mark.launch(fixture=launch_description)
def test_reached_goal(reach_goal_proc, launch_context, sim, artefacts_metrics):
    """Check that the navigation goal is reached"""

    assert_nav2_completed(reach_goal_proc, launch_context)
    goal_x, goal_y = 0.8, -0.5

    # Create waypoint pose for the navigation goal in custom_odom frame
    goal_waypoint = Pose(
        x=goal_x, y=goal_y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, frame="custom_odom"
    )

    assert_close_to_waypoint(sim, "sam_bot", goal_waypoint, threshold=0.5, export_csv=True)

