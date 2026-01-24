#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import numpy as np
import cv2
import json
import os
from ament_index_python.packages import get_package_share_directory

# ROS messages
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from cv_bridge import CvBridge
# import sensor_msgs.point_cloud2 as pc2  # ros1的写法
from sensor_msgs_py import point_cloud2 as pc2  # ros2的写法

from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class LidarVisionFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_vision_fusion_node')

        # === 参数声明 ===
        self.declare_parameter('extrinsic_json_path', '')
        # self.declare_parameter('road_class_id', 0)  # Fast-SCNN 中 road 的类别 ID
        self.declare_parameter('water_class_id', 0)  # 0: water, 1: non-water
        self.declare_parameter('min_points_in_bbox', 3)  # bbox 内至少多少点才认为有效
        self.declare_parameter('max_detection_range', 50.0)  # 最大检测距离（米）

        extrinsic_param = self.get_parameter('extrinsic_json_path').value

        if extrinsic_param:
            # 如果用户显式指定了路径（如调试时），优先使用
            extrinsic_path = extrinsic_param
        else:
            # 否则使用包内默认配置
            pkg_share = get_package_share_directory('lidar_vision_fusion')
            extrinsic_path = os.path.join(pkg_share, 'config', 'extrinsic.json')

        if not os.path.exists(extrinsic_path):
            self.get_logger().error(f"Extrinsic file not found: {extrinsic_path}")
            raise FileNotFoundError("Missing extrinsic.json")

        # self.road_class_id = self.get_parameter('road_class_id').value
        self.water_class_id = self.get_parameter('water_class_id').value
        self.min_points = self.get_parameter('min_points_in_bbox').value
        self.max_range = self.get_parameter('max_detection_range').value

        # === 加载标定参数 ===
        with open(extrinsic_path, 'r') as f:
            calib = json.load(f)
        
        # 相机内参 K (3x3)
        self.K = np.array(calib['camera_matrix'], dtype=np.float32)
        
        # 外参：rvec (旋转向量), tvec (平移向量)
        rvec = np.array(calib['rvec'], dtype=np.float32)
        tvec = np.array(calib['tvec'], dtype=np.float32)
        
        # 将 rvec 转为旋转矩阵 R (3x3)
        R, _ = cv2.Rodrigues(rvec)
        
        # 构建 4x4 齐次变换矩阵: T_lidar_to_cam = [R | t; 0 0 0 1]
        self.T_lidar_to_cam = np.eye(4, dtype=np.float32)
        self.T_lidar_to_cam[:3, :3] = R
        self.T_lidar_to_cam[:3, 3] = tvec

        self.get_logger().info("Loaded calibration parameters.")

        # === 订阅器 ===
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_detections = Subscriber(self, Detection2DArray, '/yolov5/bboxes')
        self.sub_mask = Subscriber(self, Image, '/fastscnn/segmentation')
        self.sub_pointcloud = Subscriber(self, PointCloud2, '/unilidar/cloud')
        self.sub_image = Subscriber(self, Image, '/camera/left/image_rect')  # 用于调试可视化（可选）

        # 时间同步（容忍 50ms 偏差）
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_detections, self.sub_mask, self.sub_pointcloud, self.sub_image],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.fusion_callback)

        # === 发布器 ===
        self.pub_objects = self.create_publisher(Detection3DArray, '/fused/objects', 10)
        self.pub_freespace = self.create_publisher(PointCloud2, '/fused/freespace', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fused/objects_markers', 10)

        self.bridge = CvBridge()
        self.get_logger().info("LiDAR-Vision Fusion node started.")

        # ===== Debug Marker 缓存 =====
        self.latest_marker_array = MarkerArray()
        self.marker_lock = False  # Python 下简单点，不用 mutex 也行

        # ===== 定时发布 Marker（5 Hz）=====
        self.marker_timer = self.create_timer(
            0.2,  # 5 Hz
            self.publish_markers_timer
        )


    def publish_markers_timer(self):
        # 没有数据就不发（也可以选择发空 MarkerArray）
        if not self.latest_marker_array.markers:
            return

        # 更新时间戳（非常重要！）
        now = self.get_clock()->now().to_msg()
        for m in self.latest_marker_array.markers:
            m.header.stamp = now

        self.marker_pub.publish(self.latest_marker_array)


    def fusion_callback(self, detections_msg, mask_msg, pointcloud_msg, image_msg):
        try:
            # === 1. 解析语义 mask ===
            mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')  # HxW, uint8
            h_img, w_img = mask.shape

            # === 2. 解析点云，并转换到相机坐标系 ===
            points_lidar = []
            for p in pc2.read_points(pointcloud_msg, skip_nans=True, field_names=("x", "y", "z")):
                points_lidar.append([p[0], p[1], p[2]])
            points_lidar = np.array(points_lidar, dtype=np.float32)  # Nx3

            if points_lidar.size == 0:
                return

            # 转换到相机坐标系
            points_homo = np.hstack([points_lidar, np.ones((points_lidar.shape[0], 1))])  # Nx4
            points_cam = (self.T_lidar_to_cam @ points_homo.T).T  # Nx4
            points_cam = points_cam[:, :3]  # Nx3

            # 过滤掉相机后方的点（z <= 0）
            valid = points_cam[:, 2] > 0.1
            points_cam = points_cam[valid]
            points_lidar = points_lidar[valid]

            if points_cam.size == 0:
                return

            # 投影到像素平面
            pixels = (self.K @ points_cam.T).T  # Nx3
            pixels = pixels[:, :2] / pixels[:, 2:3]  # Nx2
            pixels = np.round(pixels).astype(int)

            # 过滤出图像范围内的点
            u_valid = (pixels[:, 0] >= 0) & (pixels[:, 0] < w_img)
            v_valid = (pixels[:, 1] >= 0) & (pixels[:, 1] < h_img)
            in_image = u_valid & v_valid
            pixels = pixels[in_image]
            points_cam = points_cam[in_image]
            points_lidar = points_lidar[in_image]

            if pixels.size == 0:
                return

            # === 3. Free Space: 提取 road 点 ===
            # road_mask = (mask[pixels[:, 1], pixels[:, 0]] == self.road_class_id)
            # ground_points = points_lidar[road_mask]
            water_mask = (mask[pixels[:, 1], pixels[:, 0]] == self.water_class_id)
            free_points = points_lidar[water_mask]

            # 发布 freespace（作为点云）
            if free_points.size > 0:
                freespace_msg = PointCloud2()
                freespace_msg.header = pointcloud_msg.header  # 保持时间戳和 frame_id
                freespace_msg = pc2.create_cloud_xyz32(pointcloud_msg.header, free_points.tolist())
                self.pub_freespace.publish(freespace_msg)

            # === 4. Object Fusion: 处理每个 2D detection ===
            detections_3d = Detection3DArray()
            detections_3d.header = detections_msg.header

            marker_array = MarkerArray()
            marker_id = 0



            for det2d in detections_msg.detections:
                x_center = det2d.bbox.center.position.x
                y_center = det2d.bbox.center.position.y
                width = det2d.bbox.size_x
                height = det2d.bbox.size_y

                x1 = int(x_center - width / 2)
                y1 = int(y_center - height / 2)
                x2 = int(x_center + width / 2)
                y2 = int(y_center + height / 2)

                # 找出落在 bbox 内的点
                in_bbox = (
                    (pixels[:, 0] >= x1) & (pixels[:, 0] <= x2) &
                    (pixels[:, 1] >= y1) & (pixels[:, 1] <= y2)
                )
                obj_points_lidar = points_lidar[in_bbox]

                if len(obj_points_lidar) < self.min_points:
                    continue

                # 过滤远距离点
                distances = np.linalg.norm(obj_points_lidar, axis=1)
                if np.min(distances) > self.max_range:
                    continue

                # 计算 3D 中心（在 LiDAR 坐标系）
                center_3d = np.mean(obj_points_lidar, axis=0)

                # 创建 Detection3D
                det3d = Detection3D()
                det3d.header = detections_msg.header
                det3d.results = det2d.results  # 复用类别和置信度

                pose = Pose()
                pose.position.x = float(center_3d[0])
                pose.position.y = float(center_3d[1])
                pose.position.z = float(center_3d[2])
                pose.orientation.w = 1.0
                det3d.bbox.center = pose

                # 粗略估计尺寸（可选：用 PCA 或固定值）
                det3d.bbox.size.x = 1.0  # 可根据类别设定
                det3d.bbox.size.y = 1.0
                det3d.bbox.size.z = 1.5

                detections_3d.detections.append(det3d)

                # 创建 Marker 队列
                marker = Marker()
                marker.header = detections_3d.header
                marker.header.frame_id = pointcloud_msg.header.frame_id
                marker.ns = 'fused_objects'
                marker.id = marker_id
                marker_id += 1

                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # === 位置 ===
                # marker.pose.position.x = det2d.bbox.center.position.x
                # marker.pose.position.y = det2d.bbox.center.position.y
                # marker.pose.position.z = det2d.bbox.center.position.z
                # marker.pose.orientation = det2d.bbox.center.orientation
                marker.pose.position.x = float(center_3d[0])
                marker.pose.position.y = float(center_3d[1])
                marker.pose.position.z = float(center_3d[2])
                marker.pose.orientation.w = 1.0


                # === 尺寸 ===
                marker.scale.x = det2d.bbox.size.x
                marker.scale.y = det2d.bbox.size.y
                marker.scale.z = det2d.bbox.size.z

                # === 颜色（可按类别区分）===
                marker.color.r = 0.1
                marker.color.g = 0.8
                marker.color.b = 0.1
                marker.color.a = 0.6

                # === 生命周期（防止残影）===
                marker.lifetime = Duration(sec=0, nanosec=300_000_000)

                marker_array.markers.append(marker)


            self.pub_objects.publish(detections_3d)
            # self.marker_pub.publish(marker_array)
            # ===== 缓存最新 marker，用于定时发布 =====
            self.latest_marker_array = marker_array

        except Exception as e:
            self.get_logger().error(f"Fusion error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = LidarVisionFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()