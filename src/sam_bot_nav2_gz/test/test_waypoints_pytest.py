import os
import pytest
import rclpy
import json
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
import yaml
from datetime import datetime


ARTEFACTS_PARAMS_FILE = os.environ.get(
    "ARTEFACTS_SCENARIO_PARAMS_FILE", "scenario_params.yaml"
)

waypoints = yaml.safe_load('''
waypoints:
  - position:
      x: 0.8006443977355957
      y: 0.5491957664489746
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0055409271259092485
      w: 0.9999846489454652
  - position:
      x: 1.8789787292480469
      y: 0.5389942526817322
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.010695864295550759
      w: 0.9999427976074288
  - position:
      x: 3.0792641639709473
      y: 0.6118782758712769
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.01899610435153287
      w: 0.9998195577300264
  - position:
      x: 3.8347740173339844
      y: 0.012513279914855957
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7548200584119721
      w: 0.6559319167558071
''')
waypoints = yaml.safe_load('''
waypoints:
  - position:
      x: 0.8006443977355957
      y: 0.5491957664489746
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0055409271259092485
      w: 0.9999846489454652
  - position:
      x: 1.8789787292480469
      y: 0.5389942526817322
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.010695864295550759
      w: 0.9999427976074288
  - position:
      x: 3.0792641639709473
      y: 0.6118782758712769
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.01899610435153287
      w: 0.9998195577300264
  - position:
      x: 3.8347740173339844
      y: 0.012513279914855957
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.7548200584119721
      w: 0.6559319167558071
  - position:
      x: 3.084421157836914
      y: -0.5701640844345093
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9998472894684893
      w: 0.01747563282157926
  - position:
      x: 2.19096302986145
      y: -0.609535813331604
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.9999322787364863
      w: 0.011637780753125607
  - position:
      x: 0.8946757316589355
      y: -0.5464844703674316
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.9850211921086874
      w: 0.1724333236262069
  - position:
      x: -0.14899730682373047
      y: -0.011111736297607422
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7179595085705036
      w: 0.6960848684271199
''')

def deep_merge_dicts(source, override):
    """Recursively merge two dictionaries, with values from `override` taking precedence over `source`"""
    for key, value in override.items():
        if isinstance(value, dict) and key in source:
            source[key] = deep_merge_dicts(source[key], value)
        else:
            source[key] = value
    return source

def rosify_params(params: dict):
    """
    Store `params` in `param_file` and convert to ros2 param file nested format,
    to be used by the launch file
    """
    content = {}
    for k, v in params.items():
        try:
            node, pname = k.split("/")
        except Exception:
            print(
                localise(
                    "Problem with parameter name. Please ensure params are in the format `node/param`"
                )
            )
            return
        if node not in content:
            content[node] = {"ros__parameters": {}}
        # handles nested keys for params in the form of dot notation
        current_level = content[node]["ros__parameters"]
        keys = pname.split(".")
        for key in keys[:-1]:
            if key not in current_level:
                current_level[key] = {}
            current_level = current_level[key]
        current_level[keys[-1]] = v
    return content


def merge_ros_params_files(source, override, destination):
    """Merge two ROS2 yaml parameter files into one, overriding the values in the first one with the values in `override`"""
    import yaml

    with open(source, "r") as f:
        source_params = yaml.safe_load(f)

    with open(override, "r") as f:
        override_param = yaml.safe_load(f)
        override_params_ros = rosify_params(override_param)

    merged_params = deep_merge_dicts(source_params, override_params_ros)
    with open(destination, "w") as f:
        yaml.dump(merged_params, f)


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
            source_params_file, ARTEFACTS_PARAMS_FILE, new_params_file
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


    # Create waypoint pose for the navigation goal in custom_odom frame
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
async def test_0_nav2_started(follow_waypoints_proc, launch_context):
    """Test that Nav2 starts successfully"""

    def validate_nav2_output(output):
        print(
            f"follow_waypoints output: '{output[:200]}...' (truncated)"
            if len(output) > 200
            else f"follow_waypoints output: '{output}'"
        )
        if not output.strip():
            print("WARNING: follow_waypoints process produced no output!")
        # Wait for the complete navigation process, then check if Nav2 was activated
        assert "Nav2 active!" in output, "process never printed Nav2 active!"

    # Get the follow_waypoints process from the launch context
    print("Starting to wait for follow_waypoints process output...")
    await process_tools.assert_output(
        launch_context, follow_waypoints_proc, validate_nav2_output, timeout=120
    )
    print(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


@pytest.mark.launch(fixture=launch_description)
@pytest.mark.parametrize("waypoint_idx", range(len(waypoints["waypoints"])))
def test_1_reached_waypoint(follow_waypoints_proc, launch_context, sim, artefacts_metrics, waypoint_idx):
    """Check that each waypoint is reached"""
    #TODO, without async, check timing could be way too late

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

    def validate_goal_output(output):
        print(f"follow_waypoints output: '{output}'")
        if not output.strip():
            print("WARNING: follow_waypoints process produced no output!")
        assert "Goal succeeded!" in output, "process never printed Goal succeeded!"

