from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""
一键启动无人船ROS2项目
"""

def generate_launch_description():
    # 相机驱动
    camera_driver = Node(
        package='camera_driver',
        executable='camera_driver',  # 确保名字正确
        name='camera_driver_node',
        output='screen'
    )

    # 图像校正
    camera_rectify = Node(
        package='camera_driver',
        executable='camera_rectified',
        name='camera_rectify_node',
        output='screen'
    )

    # 双目视差
    disparity_params = os.path.join(
        get_package_share_directory('stereo_disparity'),
        'config', 'sgbm_params.yaml'
    )
    stereo_disparity = Node(
        package='stereo_disparity',
        executable='stereo_disparity',
        name='stereo_disparity_node',
        parameters=[disparity_params],
        output='screen'
    )

    # yolov5目标检测
    yolov5_detector = Node(
        package='yolov5_detector',
        executable='yolov5_detector_node',
        name='yolov5_detector_node',
        parameters=[{'model_path': '/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt'}],
        output='screen'
    )

    return LaunchDescription([
        camera_driver,
        camera_rectify,
        stereo_disparity,
        yolov5_detector,
    ])