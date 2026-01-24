from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    回灌（fillback）专用启动文件

    特点：
    1. 只启动算法节点，不启动任何传感器驱动；
    2. 所有节点 use_sim_time = True；
    3. 输入 topic 显式绑定（来自 bag）；
    4. 输出 topic 统一走 output_ns（如 debug / v2 / exp_xxx）；
    5. 原始 bag topic 完全不被污染；
    """

    # ==================== Launch Arguments ====================
    input_ns = LaunchConfiguration('input_ns')
    output_ns = LaunchConfiguration('output_ns')

    declare_input_ns = DeclareLaunchArgument(
        'input_ns',
        default_value='',
        description='Input namespace from rosbag (usually empty)'
    )

    declare_output_ns = DeclareLaunchArgument(
        'output_ns',
        default_value='debug',
        description='Output namespace for algorithm results'
    )

    # ==================== 参数路径 ====================
    disparity_params = os.path.join(
        get_package_share_directory('stereo_disparity'),
        'config', 'sgbm_params.yaml'
    )

    # ==================== 视觉：立体匹配 ====================
    stereo_disparity = Node(
        package='stereo_disparity',
        executable='stereo_disparity',
        name='stereo_disparity_node',
        parameters=[
            disparity_params,
            {'use_sim_time': True}
        ],
        remappings=[
            # ---------- 输入（来自 bag） ----------
            ('/camera/left/image_rect',   [input_ns, '/camera/left/image_rect']),
            ('/camera/right/image_rect',  [input_ns, '/camera/right/image_rect']),
            ('/stereo/left/camera_info',  [input_ns, '/camera/left/camera_info']),
            ('/stereo/right/camera_info', [input_ns, '/camera/right/camera_info']),

            # ---------- 输出（隔离） ----------
            ('/stereo/depth',            [output_ns, '/stereo/depth']),
            ('/stereo/disparity',        [output_ns, '/stereo/disparity']),
            ('/stereo/disparity_color',  [output_ns, '/stereo/disparity_color']),
        ],
        output='screen'
    )

    # ==================== 视觉：YOLOv5 ====================
    yolov5_detector = Node(
        package='yolov5_detector',
        executable='yolov5_detector_node',
        name='yolov5_detector_node',
        parameters=[
            {
                'model_path': '/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt',
                'use_sim_time': True
            }
        ],
        remappings=[
            # 输入
            ('/camera/left/image_rect', [input_ns, '/camera/left/image_rect']),

            # 输出
            ('/yolov5/bboxes',            [output_ns, '/yolov5/bboxes']),
            ('/yolov5/detections_image', [output_ns, '/yolov5/detections_image']),
        ],
        output='screen'
    )

    # ==================== 视觉：Fast-SCNN ====================
    fastscnn_segmenter = Node(
        package='fastscnn_segmenter',
        executable='fastscnn_segmenter_node',
        name='fastscnn_segmenter_node',
        parameters=[
            {
                'model_path': '/home/riba/Fast-SCNN-pytorch/weights/fast_scnn_water.pth',
                'num_classes': 2,
                'use_sim_time': True
            }
        ],
        remappings=[
            # 输入
            ('/camera/left/image_rect', [input_ns, '/camera/left/image_rect']),

            # 输出
            ('/fastscnn/segmentation',       [output_ns, '/fastscnn/segmentation']),
            ('/fastscnn/segmentation_color', [output_ns, '/fastscnn/segmentation_color']),
        ],
        output='screen'
    )

    # ==================== 激光：多雷达融合 ====================
    multi_lidar_merge = Node(
        package='vision_lidar_capture',
        executable='multi_lidar_merge',
        name='multi_lidar_merge_node',
        parameters=[
            {
                'merge_rounds': 5,
                'use_sim_time': True
            }
        ],
        remappings=[
            # 输入
            ('/unilidar/cloud', [input_ns, '/unilidar/cloud']),

            # 输出
            ('/unilidar/cloud_multi', [output_ns, '/unilidar/cloud_multi']),
        ],
        output='screen'
    )

    # ==================== 融合 ====================
    lidar_vision_fusion = Node(
        package='lidar_vision_fusion',
        executable='lidar_vision_fusion',
        name='lidar_vision_fusion_node',
        parameters=[
            {'use_sim_time': True}
        ],
        remappings=[
            # 输入
            ('/stereo/depth',             [output_ns, '/stereo/depth']),
            ('/yolov5/bboxes',            [output_ns, '/yolov5/bboxes']),
            ('/fastscnn/segmentation',    [output_ns, '/fastscnn/segmentation']),
            ('/unilidar/cloud',     [output_ns, '/unilidar/cloud']),

            # 输出
            ('/fused/freespace',          [output_ns, '/fused/freespace']),
            ('/fused/objects',            [output_ns, '/fused/objects']),
            ('/fused/objects_markers',    [output_ns, '/fused/objects_markers']),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_input_ns,
        declare_output_ns,

        stereo_disparity,
        yolov5_detector,
        fastscnn_segmenter,
        multi_lidar_merge,
        lidar_vision_fusion,
    ])
