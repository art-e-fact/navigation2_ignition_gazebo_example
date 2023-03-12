import unittest
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import pytest
from launch_testing.actions import ReadyToTest
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

@pytest.mark.launch_test
def generate_test_description():
    launch_navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("sam_bot_description"),
                    "launch",
                ),
                "/complete_navigation.launch.py",
            ]
        ),
        launch_arguments=[("gazebo_args", "-s"), ("use_rivz", "false")],
    )

    return LaunchDescription(
        [
            launch_navigation_stack,
            ReadyToTest(),
        ]
    )


class TestBringup(unittest.TestCase):
    def test_arm(self, proc_output):
        # This will match stdout from test_process.
        proc_output.assertWaitFor("Ready for navigation!", timeout=180)