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

def main():
    rclpy.init()
    navigator = BasicNavigator()

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
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

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
