import launch
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.events.process import ProcessIO
from launch.event_handlers import OnProcessIO

# Create event handler that waits for an output message and then returns actions
def on_matching_output(matcher: str, result: launch.SomeActionsType):
    def on_output(event: ProcessIO):
        for line in event.text.decode().splitlines():
            if matcher in line:
                return result

    return on_output


# Lanch the robot and the navigation stack


def generate_launch_description():
    # Messages are from: https://navigation.ros.org/setup_guides/sensors/setup_sensors.html#launching-nav2
    diff_drive_loaded_message = (
        "Sucessfully loaded controller diff_drive_base_controller into state active"
    )
    toolbox_ready_message = "Registering sensor"
    navigation_ready_message = "Creating bond timer"

    bringup = ExecuteProcess(
        name="launch_bringup",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("sam_bot_description"),
                    "launch",
                    "display.launch.py",
                ]
            ),
            "use_rviz:=false",
            PythonExpression([
                "'' if '",
                LaunchConfiguration("gz_args"),
                "' == ''",
                " else 'gz_args:=",
                LaunchConfiguration("gz_args"), "'"
            ])
        ],
        output="screen",
    )
    toolbox = ExecuteProcess(
        name="launch_slam_toolbox",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_async_launch.py",
                ]
            ),
        ],
        output="screen",
    )
    waiting_toolbox = RegisterEventHandler(
        OnProcessIO(
            target_action=bringup,
            on_stdout=on_matching_output(
                diff_drive_loaded_message,
                [
                    LogInfo(
                        msg="Diff drive controller loaded. Starting SLAM Toolbox..."
                    ),
                    toolbox,
                ],
            ),
        )
    )

    navigation = ExecuteProcess(
        name="launch_navigation",
        cmd=[
            "ros2",
            "launch",
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=launch.conditions.IfCondition(LaunchConfiguration("use_rviz")),
    )
    waiting_navigation = RegisterEventHandler(
        OnProcessIO(
            target_action=toolbox,
            on_stdout=on_matching_output(
                toolbox_ready_message,
                [
                    LogInfo(msg="SLAM Toolbox loaded. Starting navigation..."),
                    # TODO Debug: Navigation fails to start if it's launched right after the slam_toolbox
                    TimerAction(
                        period=20.0,
                        actions=[navigation],
                    ),
                    rviz_node,
                ],
            ),
        )
    )

    waiting_success = RegisterEventHandler(
        OnProcessIO(
            target_action=navigation,
            on_stdout=on_matching_output(
                navigation_ready_message,
                [
                    LogInfo(msg="Ready for navigation!"),
                ],
            ),
        )
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=[
                    FindPackageShare("sam_bot_description"),
                    "/rviz/navigation_config.rviz",
                ],
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                "gz_args",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Absolute path to rviz config file",
            ),
            bringup,
            waiting_toolbox,
            waiting_navigation,
            waiting_success,
        ]
    )
