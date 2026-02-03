from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

simulation_flag = True

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # params = "src/welaboat/welaboat_bringup/welaboat_bringup/config/nav2_minimal.yaml"
    params = "src/welaboat/welaboat_bringup/welaboat_bringup/config/nav2_param.yaml"

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'default_bt_xml_filename': 'navigate_to_pose_w_replanning_and_recovery.xml', # 强制覆盖参数文件
            }],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator',
                    'behavior_server',
                    'velocity_smoother'
                ]
            }],
        ),
    ])
