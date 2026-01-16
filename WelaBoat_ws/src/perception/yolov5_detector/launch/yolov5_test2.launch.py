from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov5_detector',
            executable='yolov5_detector_node',
            name='yolov5_detector_node'
        )
    ])
