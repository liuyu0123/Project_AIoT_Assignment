import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #找到 URDF 文件
    urdf_tutorial_path = get_package_share_directory('fishbot_description')
    fishbot_model_path = urdf_tutorial_path + '/urdf/fishbot.urdf'

    #声明启动参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(fishbot_model_path),
        description='URDF绝对路径')

    #把 URDF 读成 字符串参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['cat ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)

    #启动两个节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    #打包成 LaunchDescription
    launch_desp = launch.LaunchDescription(
        [
            action_declare_arg_mode_path,
            joint_state_publisher_node,
            robot_state_publisher_node,
        ]
    )

    return launch_desp