import launch
import launch_ros

def generate_launch_description():
    action_declare_arg_add_a = launch.actions.DeclareLaunchArgument(
        'talker.weight_a',
        default_value='1.0',
        description='add factor: a'
    )
    action_declare_arg_add_b = launch.actions.DeclareLaunchArgument(
        'talker.weight_b',
        default_value='1.0',
        description='add factor: b'
    )
    action_declare_listener_add_a = launch.actions.DeclareLaunchArgument(
        'listener.a',
        default_value='5.0',
        description='add : a'
    )
    action_declare_listener_add_b = launch.actions.DeclareLaunchArgument(
        'listener.b',
        default_value='5.0',
        description='add : b'
    )


    action_node_talker = launch_ros.actions.Node(
        package='add_test_srv',
        executable='talker',
        name='add_talker_node_srv',
        output='screen',
        parameters=[{
            'weight_a': launch.substitutions.LaunchConfiguration('talker.weight_a'),
            'weight_b': launch.substitutions.LaunchConfiguration('talker.weight_b'),
        }],
    )

    action_node_listener = launch_ros.actions.Node(
        package='add_test_srv',
        executable='listener',
        name='add_listener_node_srv',
        output='screen',
        parameters=[],
        arguments=[
            launch.substitutions.LaunchConfiguration('listener.a'),
            launch.substitutions.LaunchConfiguration('listener.b'),
        ],
    )

    #动作4-组织动作成组，把多个动作放到一组
    action_group = launch.actions.GroupAction([
        #动作5-定时器
        launch.actions.TimerAction(period=1.0, actions=[action_node_talker]),
        launch.actions.TimerAction(period=2.0, actions=[action_node_listener]),
    ])


    return launch.LaunchDescription([
        action_declare_arg_add_a,
        action_declare_arg_add_b,
        action_declare_listener_add_a,
        action_declare_listener_add_b,
        # action_node_talker,
        # action_node_listener,
        # 控制启动顺序，方法1：组合并结合 TimerAction
        action_group,
    ])