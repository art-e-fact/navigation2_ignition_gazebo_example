#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_outdoor_example')

    #TODO: Replace with robot model and robot_publisher
    # spawn = Node( package='ros_gz_sim', executable='create', arguments=[ '-name', 'ROBOT_NAME', '-topic', 'robot_description', ], output='screen' ) 

    gnss_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_transform',
        output='screen',
        arguments = "--x 0 --y 0 --z 1.0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id vehicle_blue/gnss/navsat".split(' '),
        )

    imu_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_transform',
        output='screen',
        arguments = "--x 0 --y 0 --z 0.5 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id imu_link".split(' '),
        )

    #END TODO
    
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments = "--x 1 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        )
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
            "publish_filtered_gps": False,
            "broadcast_utm_transform": False,
        }])

    ukf_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ukf.yaml')])

    return LaunchDescription(
        [
            ukf_localization_node,
            navsat_transform_node,
            map_transform_node,
            gnss_transform_node,
            imu_transform_node,
        ]
    )


