import launch
import launch_ros

def generate_launch_description():
    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        name='turtle_control_node',
        output='screen',
        parameters=[
        ]
    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        name='patrol_client_node',
        output='log',
        parameters=[
        ]
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='both',
        parameters=[
        ]
    )
    launch_description = launch.LaunchDescription([
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim_node,
    ])
    return launch_description