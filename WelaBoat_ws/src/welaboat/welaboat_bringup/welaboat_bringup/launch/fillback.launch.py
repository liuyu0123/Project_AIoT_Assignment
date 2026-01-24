from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    回灌专用启动文件：
    1. 只启动“算法”节点，不启动任何驱动；
    2. 所有输出 topic 统一加 _debug 后缀，方便 ros2 bag record 过滤；
    3. 参数路径保持与 welaboat.launch.py 一致，可直接复用。
    """

    # ------------- 视觉部分 -------------
    disparity_params = os.path.join(
        get_package_share_directory('stereo_disparity'),
        'config', 'sgbm_params.yaml'
    )

    stereo_disparity = Node(
        package='stereo_disparity',
        executable='stereo_disparity',
        name='stereo_disparity_node',
        parameters=[disparity_params],
        remappings=[
            ('/stereo/depth', '/stereo/depth_debug'),
            ('/stereo/disparity', '/stereo/disparity_debug'),
            ('/stereo/disparity_color', '/stereo/disparity_color_debug'),
        ],
        output='screen'
    )

    yolov5_detector = Node(
        package='yolov5_detector',
        executable='yolov5_detector_node',
        name='yolov5_detector_node',
        parameters=[{'model_path': '/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt'}],
        remappings=[
            ('/yolov5/bboxes', '/yolov5/bboxes_debug'),
            ('/yolov5/detections_image', '/yolov5/detections_image_debug'),
        ],
        output='screen'
    )

    fastscnn_segmenter = Node(
        package='fastscnn_segmenter',
        executable='fastscnn_segmenter_node',
        name='fastscnn_segmenter_node',
        parameters=[{'model_path': '/home/riba/Fast-SCNN-pytorch/weights/fast_scnn_water.pth',
                     'input_topic': '/camera/left/image_rect',
                     'num_classes': 2}],
        remappings=[
            ('/fastscnn/segmentation', '/fastscnn/segmentation_debug'),
            ('/fastscnn/segmentation_color', '/fastscnn/segmentation_color_debug'),
        ],
        output='screen'
    )

    # ------------- 激光部分 -------------
    multi_lidar_merge = Node(
        package='vision_lidar_capture',
        executable='multi_lidar_merge',
        name='multi_lidar_merge_node',
        parameters=[{'merge_rounds': 5}],
        remappings=[('/unilidar/cloud_multi', '/unilidar/cloud_multi_debug')],
        output='screen'
    )

    # ------------- 融合节点 -------------
    lidar_vision_fusion = Node(
        package='lidar_vision_fusion',
        executable='lidar_vision_fusion',
        name='lidar_vision_fusion_node',
        remappings=[
            ('/fused/freespace', '/fused/freespace_debug'),
            ('/fused/objects', '/fused/objects_debug'),
            ('/fused/objects_markers', '/fused/objects_markers_debug'),
        ],
        output='screen'
    )

    return LaunchDescription([
        stereo_disparity,
        yolov5_detector,
        fastscnn_segmenter,
        multi_lidar_merge,
        lidar_vision_fusion
    ])