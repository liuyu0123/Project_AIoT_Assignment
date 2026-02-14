from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_path = get_package_share_directory('welaboat_localization')

    ekf_local_config = os.path.join(pkg_path, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_path, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_path, 'config', 'navsat.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        # ----------------------------
        # 0. 首先发布静态 TF: base_link -> gps_link
        #    假设 GPS 安装在机器人中心正上方 0.5m 处
        # ----------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_tf_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'gps_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            # 格式: x y z yaw pitch roll parent_frame child_frame
            # 或者使用四元数: x y z qx qy qz qw parent_frame child_frame
        ),


        # ----------------------------
        # 1. 首先启动 EKF Local (world_frame: odom)
        #    它发布 /odometry/filtered (即 odom->base_link)
        # ----------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/mavros/imu/data'),
                ('odometry/filtered', '/odometry/filtered')  # 明确指定输出话题
            ]
        ),

        # ----------------------------
        # 2. 然后启动 NavSat Transform
        #    它订阅 /odometry/filtered (来自 ekf_local)
        #    发布 /odometry/gps
        # ----------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/mavros/imu/data'),
                ('gps/fix', '/mavros/global_position/raw/fix'),
                ('odometry/filtered', '/odometry/filtered'),  # 来自 ekf_local
                ('odometry/gps', '/odometry/gps')
            ]
        ),

        # ----------------------------
        # 3. 最后启动 EKF Global (world_frame: map)
        #    它订阅 /odometry/gps (来自 navsat_transform)
        #    发布 map->odom
        # ----------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_config,
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu/data', '/mavros/imu/data'),
                ('odometry/filtered', '/odometry/filtered/global')  # 避免与 local 冲突
            ]
        ),
    ])