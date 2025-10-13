import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取配置文件路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 创建 launch 参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = launch.substitutions.LaunchConfiguration('map', default=os.path.join(fishbot_navigation2_dir, 'map', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration('param_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='map', default_value=map_yaml_file,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path, 
                                             description='Full path to the ROS2 parameters file to use'),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                            'map': map_yaml_file,
                            'use_sim_time': use_sim_time,
                            'params_file': nav2_param_path,
            }.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])