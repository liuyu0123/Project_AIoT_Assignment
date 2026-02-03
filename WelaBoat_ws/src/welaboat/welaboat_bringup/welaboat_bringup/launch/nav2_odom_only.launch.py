from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression

simulation_flag = True

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    # 单独为 bt_navigator 创建参数
    bt_navigator_params = {
        'use_sim_time': use_sim_time,
        'global_frame': 'odom',
        'robot_base_frame': 'base_link',
        'bt_loop_duration': 10,
        # 'default_bt_xml_filename': 'follow_point.xml',
        'default_nav_to_pose_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml',
        'default_nav_through_poses_bt_xml': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/follow_point.xml',
        'goal_blackboard_id': 'goal',
        'goals_blackboard_id': 'goals',
        'path_blackboard_id': 'path',
        'plugin_lib_names': [
            'nav2_compute_path_to_pose_action_bt_node',
            'nav2_follow_path_action_bt_node',
            'nav2_back_up_action_bt_node',
            'nav2_spin_action_bt_node',
            'nav2_wait_action_bt_node',
            'nav2_clear_costmap_service_bt_node',
            'nav2_is_stuck_condition_bt_node',
            'nav2_goal_reached_condition_bt_node',
            'nav2_initial_pose_received_condition_bt_node',
            'nav2_rate_controller_bt_node',
            'nav2_distance_controller_bt_node',
            'nav2_speed_controller_bt_node',
            'nav2_truncate_path_action_bt_node',
            'nav2_pipeline_sequence_bt_node',
            'nav2_goal_updater_node_bt_node',
        ]
        # 'plugin_lib_names':[
        #     'nav2_compute_path_to_pose_action_bt_node',
        #     'nav2_follow_path_action_bt_node',
        #     'nav2_back_up_action_bt_node',
        #     'nav2_spin_action_bt_node',
        #     'nav2_wait_action_bt_node',
        #     'nav2_clear_costmap_service_bt_node',
        #     'nav2_is_stuck_condition_bt_node',
        #     'nav2_goal_reached_condition_bt_node',
        #     'nav2_initial_pose_received_condition_bt_node',
        #     'nav2_goal_updated_condition_bt_node',
        #     'nav2_reinitialize_global_localization_service_bt_node',
        #     'nav2_rate_controller_bt_node',
        #     'nav2_distance_controller_bt_node',
        #     'nav2_speed_controller_bt_node',
        #     'nav2_recovery_node_bt_node',
        #     'nav2_pipeline_sequence_bt_node',
        #     'nav2_round_robin_node_bt_node',
        #     'nav2_transform_available_condition_bt_node',
        #     'nav2_time_expired_condition_bt_node',
        #     'nav2_distance_traveled_condition_bt_node',
        #     'nav2_single_trigger_bt_node',
        # ]
    }


    # params = "src/welaboat/welaboat_bringup/welaboat_bringup/config/nav2_param.yaml"
    params = "/home/riba/GitProject/LIUYU/WelaBoat_ws/src/welaboat/welaboat_bringup/welaboat_bringup/config/nav2_param.yaml"

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

        # 关键修改：使用完整的字典作为 parameters
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[
        #         params,
        #         {
        #             'use_sim_time': use_sim_time,
        #             # 'default_bt_xml_filename': 'navigate_to_pose_w_replanning_and_recovery.xml',
        #             # 'default_bt_xml_filename': 'nfollow_point.xml',
        #             'default_bt_xml_filename': LaunchConfiguration(
        #                 'default_bt_xml_filename',
        #                 default='navigate_to_pose_w_replanning_and_recovery.xml'
        #             ),
        #         }
        #     ],
        # ),

        # 关键修改：不使用 YAML 文件，直接提供参数
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_params],
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