from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['conda', 'activate', 'yolov5_env'],
            shell=True
        ),
        Node(
            package='yolov5_detector',
            executable='yolov5_detector_node',
            name='yolov5_detector_node'
        ),
        # ExecuteProcess(
        #     cmd=['conda', 'activate', 'fastscnn_env'],
        #     shell=True
        # ),
        # Node(
        #     package='fastscnn_segmentor',
        #     executable='node_name',
        #     name='fastscnn_segmentor_node'
        # )
    ])