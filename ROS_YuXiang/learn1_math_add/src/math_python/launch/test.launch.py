import launch
import launch_ros


def generate_launch_description():
    action_node_talker = launch_ros.actions.Node(
        package='math_python',
        executable='talker',
        name='talker_node',
        output='screen',
        parameters=[
        ]
    )

    return launch.LaunchDescription([
        action_node_talker,
    ])