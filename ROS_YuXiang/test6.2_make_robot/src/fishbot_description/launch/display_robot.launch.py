import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    default_model_path = urdf_tutorial_path + '/urdf/first_robot.urdf'
    # 声明launch参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='mode_path',
        default_value=str(default_model_path),
        description='the path of model urdf'
    )
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('mode_path')]),
        value_type=str)
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_description}
        ]
    )
    # 关节状态发布节点
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[
        ]
    )
    # RViz节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='both',
        parameters=[
        ]
    )
    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
    
