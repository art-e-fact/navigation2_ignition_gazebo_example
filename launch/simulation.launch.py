#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Start simulation
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('nav2_outdoor_example')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' -r ' + pkg_share + '/world/diff_drive.sdf'
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat', 
                   '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )

    return LaunchDescription(
        [
            gz_sim,
            bridge,
        ]
    )


