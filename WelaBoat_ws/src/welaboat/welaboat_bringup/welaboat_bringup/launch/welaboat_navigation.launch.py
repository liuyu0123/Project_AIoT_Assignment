from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_welaboat = get_package_share_directory('welaboat_bringup')

    map_yaml = os.path.join(
        pkg_welaboat,
        'map',
        'map.yaml'
    )

    nav2_launch = os.path.join(
        pkg_welaboat,
        'launch',
        'nav2_odom_only.launch.py'
    )

    rviz_config = os.path.join(
        pkg_welaboat,
        'rviz',
        'simulation.rviz'
    )

    return LaunchDescription([

        # ------------------------
        # Map Server
        # ------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}],
        ),

        # ------------------------
        # Lifecycle Manager (for map)
        # ------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        # ------------------------
        # Fake Odom
        # ------------------------
        Node(
            package='simu_localization',
            executable='fake_odom_control',
            name='fake_odom_control',
            output='screen'
        ),

        # ------------------------
        # Static TF: map -> odom
        # ------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        ),

        # ------------------------
        # Nav2
        # ------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch)
        ),

        # ------------------------
        # RViz
        # ------------------------
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     output='screen'
        # ),

    ])
