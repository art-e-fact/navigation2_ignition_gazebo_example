#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
from rosgraph_msgs.msg import Clock


from ament_index_python.packages import get_package_share_directory
import os

package_dir = get_package_share_directory('sam_bot_nav2_gz')
waypoints_file = os.path.join(package_dir, 'test', 'waypoints.yaml')
with open(waypoints_file, 'r') as f:
    waypoints = yaml.safe_load(f)

sim_time = 0.0

def update_sim_time(msg):
    global sim_time
    sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.create_subscription(Clock, '/clock', lambda msg: update_sim_time(msg), 10)

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    goal_poses = list(map(create_pose, waypoints["waypoints"]))


    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")
    print('Nav2 active!')

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    current_wp = -1
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            if current_wp >= 0 and current_wp != feedback.current_waypoint:
                print(sim_time)
                print(navigator.get_clock().now())
                print(f"Reached waypoint: {current_wp} @{sim_time}")
            current_wp = feedback.current_waypoint
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    print(f"Reached waypoint: {current_wp} @{sim_time}")
    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
