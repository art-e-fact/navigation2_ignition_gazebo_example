# Copyright 2021 Open Source Robotics Foundation, Inc.
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
import launch

import launch_pytest
from launch_pytest.tools import process as process_tools
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

import pytest

from artefacts_toolkit.rosbag import rosbag, image_topics
from artefacts_toolkit.chart import make_chart
from artefacts_toolkit.config import get_artefacts_param
from artefacts_toolkit.gazebo import gz


@pytest.fixture
def rosbag_recorder_fixture():
    """Create and return a rosbag recorder for the test."""
    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]
    
    bag_recorder, rosbag_filepath = rosbag.get_bag_recorder(
        topics + sim_topics + metrics + camera_topics, use_sim_time=True
    )
    print(f"Recording to {rosbag_filepath}")
    return bag_recorder, rosbag_filepath


@pytest.fixture
def launch_navigation_stack():

    try:
        world = get_artefacts_param("launch", "world")
    except FileNotFoundError:
        world = "empty.world"
    # Launch a process to test
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("sam_bot_nav2_gz"),
                    "launch",
                    "waypoint_follower_example_launch.py"
                ),
            ]
        ),
        launch_arguments=[("run_headless", "True"), ("world_file", world)],
    )


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description(rosbag_recorder_fixture, launch_navigation_stack):
    """Launch a simple process to print 'hello_world'."""

        
    bag_recorder, rosbag_filepath = rosbag_recorder_fixture

     
    return launch.LaunchDescription([
        launch_navigation_stack,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])


@pytest.mark.launch(fixture=launch_description)
def test_read_stdout(launch_navigation_stack, launch_context):
    """Check if 'hello_world' was found in the stdout."""
    def validate_output(output):
        # this function can use assertions to validate the output or return a boolean.
        # pytest generates easier to understand failures when assertions are used.
        assert output.splitlines() == ['Goal succeeded!'], 'process never printed Goal succeeded!'
    process_tools.assert_output_sync(
        launch_context, launch_navigation_stack, validate_output, timeout=5)

    def validate_output(output):
        return output == 'this will never happen'
    assert not process_tools.wait_for_output_sync(
        launch_context, launch_navigation_stack, validate_output, timeout=100.1)
    yield
    # this is executed after launch service shutdown
    # assert launch_navigation_stack.return_code == 0
