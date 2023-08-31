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


    # Start GUI
    rviz_config_path = os.path.join(pkg_share, "rviz/navigation_config.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )


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



    #TODO: Replace with robot model and robot_publisher
    # spawn = Node( package='ros_gz_sim', executable='create', arguments=[ '-name', 'ROBOT_NAME', '-topic', 'robot_description', ], output='screen' ) 

    gnss_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gnss_transform',
        output='screen',
        arguments = "--x 0 --y 0 --z 1.0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id gnss_link".split(' '),
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



    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch/navigation_launch.py')),
        launch_arguments={'use_sim_time': 'False', 'params_file': os.path.join(pkg_share, 'config/nav2_params.yaml')}.items(),
    )



    return LaunchDescription(
        [
            gz_sim,
            bridge,
            rviz_node,
            map_server_node,
            map_server_lifecycle_node,
            ukf_localization_node,
            navsat_transform_node,
            map_transform_node,
            gnss_transform_node,
            imu_transform_node,
            nav2_bringup_launch
        ]
    )


