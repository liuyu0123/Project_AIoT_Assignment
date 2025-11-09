import launch
import launch_ros


def generate_launch_description():
    action_node_talker = launch_ros.actions.Node(
        package='add_test',
        executable='talker',
        name='add_talker_node',
        output='screen',
        parameters=[
        ]
    )
    action_node_listener = launch_ros.actions.Node(
        package='add_test',
        executable='listener',
        name='add_listener_node',
        output='screen',
        parameters=[
        ]
    )

    return launch.LaunchDescription([
        action_node_talker,
        action_node_listener,
    ])