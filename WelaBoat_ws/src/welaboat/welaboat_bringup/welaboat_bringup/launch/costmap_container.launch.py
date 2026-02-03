from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. 定义容器
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
    )

    # 2. 定义要加载的 Costmap 组件
    costmap_component = ComposableNode(
        package='nav2_costmap_2d',
        plugin='nav2_costmap_2d::CostmapServer',
        name='local_costmap',
        namespace='/local_costmap',
        # 注意：这里建议使用绝对路径或 launch 文件替换机制，否则可能找不到文件
        parameters=["src/navigation/config/costmap_server_component.yaml"], 
        remappings=[('...', '...')],
    )

    # 3. 将组件加载到容器中
    # 【关键修改】：将 'composable_nodes' 改为 'composable_node_descriptions'
    loader = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=[costmap_component],
    )

    return LaunchDescription([container, loader])
