from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument('model_path', default_value='')
    conda_env_arg = DeclareLaunchArgument('conda_env', default_value='yolov5_env')
    input_topic_arg = DeclareLaunchArgument('input_topic', default_value='/camera/left/image_rect')

    model_path = LaunchConfiguration('model_path')
    conda_env = LaunchConfiguration('conda_env')
    input_topic = LaunchConfiguration('input_topic')

    # ✅ 使用 conda run（关键！）
    cmd = [
        'conda', 'run', '--no-capture-output', '-n', conda_env,
        'ros2', 'run', 'yolov5_detector', 'yolov5_detector_node',
        '--ros-args',
        '-p', ['model_path:=', model_path],
        '-p', ['input_topic:=', input_topic]
    ]

    yolov5_node = ExecuteProcess(
        cmd=cmd,
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        conda_env_arg,
        input_topic_arg,
        yolov5_node
    ])