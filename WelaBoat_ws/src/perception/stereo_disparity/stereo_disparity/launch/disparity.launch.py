# stereo_disparity/stereo_disparity/launch/disparity.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的 share 目录路径（包含 config/ 和 launch/）
    pkg_share = get_package_share_directory('stereo_disparity')
    
    # 构建参数文件路径
    params_file = os.path.join(pkg_share, 'config', 'sgbm_params.yaml')

    # 创建视差节点
    disparity_node = Node(
        package='stereo_disparity',
        executable='stereo_disparity',
        name='stereo_disparity_node',
        parameters=[params_file],
        output='screen',
        emulate_tty=True  # 保留彩色日志
    )

    return LaunchDescription([
        disparity_node
    ])