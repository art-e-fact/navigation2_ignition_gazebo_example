#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = get_package_share_directory('nav2_outdoor_example')

    # Start map, localization, and navigation
    lifecycle_nodes = ['map_server']
    map_server_node = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[os.path.join(pkg_share, 'config/nav2_params.yaml')],
                arguments=['--ros-args', '--log-level', 'info'])

    map_server_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': False},
                            {'autostart': True},
                            {'node_names': lifecycle_nodes}])

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'False', 'params_file': os.path.join(pkg_share, 'config/nav2_params.yaml')}.items(),
    )



    return LaunchDescription(
        [
            map_server_node,
            map_server_lifecycle_node,
            nav2_bringup_launch
        ]
    )


