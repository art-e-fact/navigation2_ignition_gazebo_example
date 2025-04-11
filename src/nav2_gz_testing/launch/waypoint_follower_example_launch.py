# Copyright (c) 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configuration variables
    world_file_name = LaunchConfiguration("world_file")
    waypoints_path = LaunchConfiguration("waypoints_path")

    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    sim_dir = get_package_share_directory("nav2_minimal_tb4_sim")
    desc_dir = get_package_share_directory("nav2_minimal_tb4_description")
    test_pkg_dir = get_package_share_directory("nav2_gz_testing")

    robot_sdf = os.path.join(desc_dir, "urdf", "standard", "turtlebot4.urdf.xacro")
    world = PathJoinSubstitution([test_pkg_dir, "worlds", world_file_name])

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to execute gzclient)"
    )

    declare_world_file_cmd = DeclareLaunchArgument(
        name="world_file",
        default_value="bookstore.sdf",
        description="Name of the world file to load",
    )

    declare_waypoints_path_cmd = DeclareLaunchArgument(
        name="waypoints_path",
        description="Path to the waypoints YAML",
        default_value=PathJoinSubstitution([test_pkg_dir, "waypoints", "bookstore.yaml"]),
    )

    # start the simulation
    world_sdf = tempfile.mktemp(prefix="nav2_", suffix=".sdf")
    print(
        f"Temporary world file created at {world_sdf}. This will be removed on shutdown."
    )
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-r -s ", world_sdf]}.items(),
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "worlds")
    )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["not ", headless])),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, "launch", "spawn_tb4.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "robot_sdf": robot_sdf,
            "x_pose": "0.0",
            "y_pose": "0.0",
            "z_pose": "0.0",
            "roll": "0.0",
            "pitch": "0.0",
            "yaw": "0.0",
        }.items(),
    )

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": Command(["xacro", " ", robot_sdf]),
            }
        ],
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
        ),
        condition=IfCondition(PythonExpression([use_rviz, " and not ", headless])),
        launch_arguments={"namespace": ""}.items(),
    )

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={"slam": "True"}.items(),
    )

    # start the demo autonomy task
    demo_cmd = Node(
        package="nav2_gz_testing",
        executable="example_waypoint_follower.py",
        emulate_tty=True,
        output="screen",
        parameters=[
            {
                "waypoints_path": waypoints_path,
            }
        ],
    )

    # Register an event handler to shutdown everything when the demo node exits
    shutdown_on_demo_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=demo_cmd,
            on_exit=[
                EmitEvent(event=Shutdown(reason="Waypoint follower demo completed"))
            ],
        )
    )

    set_env_vars_resources2 = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(Path(os.path.join(desc_dir)).parent.resolve())
    )

    gz_models_path = ":".join([test_pkg_dir, os.path.join(test_pkg_dir, "models")])
    gz_resource_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_models_path,
    )

    ld = LaunchDescription()
    ld.add_action(gz_resource_env)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_waypoints_path_cmd)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    ld.add_action(shutdown_on_demo_exit)
    return ld
