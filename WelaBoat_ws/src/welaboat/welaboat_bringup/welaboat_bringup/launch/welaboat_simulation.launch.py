from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

"""
一键启动无人船ROS2项目
"""

def generate_launch_description():
    WelaBoat = LaunchConfiguration('WelaBoat')
    System = LaunchConfiguration('System')
    
    welaboat = DeclareLaunchArgument(
        'WelaBoat',
        default_value='/home/riba/GitProject/LIUYU/WelaBoat_ws',
        # default_value='/home/nvidia/Documents/LIUYU/WelaBoat_ws',
        description='WelaBoat workspace root'
    )
    system = DeclareLaunchArgument(
        'System',
        # default_value='/home/riba',
        default_value='/home/nvidia',
        description='WelaBoat workspace root'
    )

    # 相机驱动
    camera_driver = Node(
        package='camera_driver',
        executable='camera_driver',  # 确保名字正确
        name='camera_driver_node',
        output='screen'
    )

    # 激光雷达驱动
    lidar_driver = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'port': '/dev/ttyUSB0'},
                {'rotate_yaw_bias': 0.0},
                {'range_scale': 0.001},
                {'range_bias': 0.0},
                {'range_max': 50.0},
                {'range_min': 0.0},
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'cloud_scan_num': 18},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"}]
    )

    # 激光点云接口转换
    pointcloud_to_laserscan_convert = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        parameters=[{
            # 输入点云话题（注意：不是 'cloud_in'，是 'cloud_topic' 或直接用 remapping）
            'target_frame': 'base_link',      # 目标坐标系
            'transform_tolerance': 0.01,       # TF变换容差
            'min_height': -0.3,               # 最小高度
            'max_height': 0.3,                # 最大高度
            'angle_min': -3.14159,            # 最小角度（-π）
            'angle_max': 3.14159,             # 最大角度（π）
            'angle_increment': 0.00435,       # 角度增量（约0.25度）
            'scan_time': 0.1,                 # 扫描时间
            'range_min': 0.0,                 # 最小距离
            'range_max': 50.0,                # 最大距离
            'use_inf': True,                  # 是否使用无穷大
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', '/unilidar/cloud'),   # 输入点云话题映射
            ('scan', '/scan'),                  # 输出激光扫描话题映射
        ]
    )

    unilidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='unilidar_static_tf_node',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link',
            'unilidar_lidar'
        ],
    )

    multi_lidar_merge = Node(
      package='vision_lidar_capture',
      executable='multi_lidar_merge',
      name='multi_lidar_merge_node',
      output='screen',
      parameters=[
                {'merge_rounds': 5},
      ]
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
        # parameters=[{'model_path': '/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt'}],
        parameters=[{'model_path': [LaunchConfiguration('WelaBoat'),'/src/perception/yolov5_detector/model/yolov5s.pt']}],
        output='screen'
    )

    # fastscnn语义分割
    fastscnn_segmenter_citys = Node(
        package='fastscnn_segmenter',
        executable='fastscnn_segmenter_node',
        name='fastscnn_segmenter_node',
        parameters=[{
            # 'model_path': '/home/riba/Fast-SCNN-pytorch/weights/fast_scnn_citys.pth',
            'model_path': [LaunchConfiguration('System'),'/Fast-SCNN-pytorch/weights/fast_scnn_citys.pth'],
            'input_topic': '/camera/left/image_rect',
            'num_classes': 19
        }],
        output='screen'
    )
    fastscnn_segmenter_water = Node(
        package='fastscnn_segmenter',
        executable='fastscnn_segmenter_node',
        name='fastscnn_segmenter_node',
        parameters=[{
            # 'model_path': '/home/riba/Fast-SCNN-pytorch/weights/fast_scnn_water.pth',
            'model_path': [LaunchConfiguration('System'),'/Fast-SCNN-pytorch/weights/fast_scnn_water.pth'],
            'input_topic': '/camera/left/image_rect',
            'num_classes': 2
        }],
        output='screen'
    )

    lidar_vision_fusion = Node(
        package='lidar_vision_fusion',
        executable='lidar_vision_fusion',
        name='lidar_vision_fusion_node',
        output='screen'
    )

    fake_odom_node = Node(
        package='simu_localization',
        executable='fake_odom',
        name='fake_odom_node',
        output='screen'
    )


    return LaunchDescription([
        welaboat,
        system,
        # camera_driver,
        lidar_driver,
        pointcloud_to_laserscan_convert,
        unilidar_static_tf,
        multi_lidar_merge,
        # camera_rectify,
        # stereo_disparity,
        # yolov5_detector,
        # fastscnn_segmenter_citys,
        # fastscnn_segmenter_water,
        # lidar_vision_fusion,
        fake_odom_node,
    ])