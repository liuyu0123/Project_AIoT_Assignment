import launch
import launch_ros


def generate_launch_description():
    action_declare_arg_add_a = launch.actions.DeclareLaunchArgument(
        'weight_factor_a',
        default_value='1.0',
        description='add factor: a'
    )
    action_declare_arg_add_b = launch.actions.DeclareLaunchArgument(
        'weight_factor_b',
        default_value='1.0',
        description='add factor: b'
    )

    action_node_talker = launch_ros.actions.Node(
        package='add_test_srv',
        executable='talker',
        name='add_talker_node_srv',
        output='screen',
        parameters=[{
            'weight_a': launch.substitutions.LaunchConfiguration('weight_factor_a', default='1.0'),
            'weight_b': launch.substitutions.LaunchConfiguration('weight_factor_b', default='1.0'),
        }],
    )

    action_node_listener = launch_ros.actions.Node(
        package='add_test_srv',
        executable='listener',
        name='add_listener_node_srv',
        output='screen',
        parameters=[],
    )

    return launch.LaunchDescription([
        action_declare_arg_add_a,
        action_declare_arg_add_b,
        action_node_talker,
        action_node_listener,
    ])