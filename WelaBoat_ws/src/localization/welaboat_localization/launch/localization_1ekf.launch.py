from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_path = get_package_share_directory('welaboat_localization')
    navsat_config = os.path.join(pkg_path, 'config', 'navsat.yaml')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        # 静态 TF: base_link -> gps_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_tf_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'gps_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # NavSat Transform: GPS -> /odometry/gps
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/mavros/imu/data'),
                ('gps/fix', '/mavros/global_position/raw/fix'),
                # ('odometry/filtered', '/odometry/filtered'),  # 来自 EKF 的反馈
                ('odometry/gps', '/odometry/gps')
            ]
        ),

        # 单层 EKF: 融合 GPS + IMU -> map -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/mavros/imu/data'),
                ('odometry/gps', '/odometry/gps'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),
    ])