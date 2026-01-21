#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np
import open3d as o3d
import struct


class MultiScanMerge(Node):
    def __init__(self):
        super().__init__('multi_scan_merge')

        # 参数：拼几圈
        self.declare_parameter('merge_rounds', 5)
        self.declare_parameter('output_topic', '/unilidar/cloud_multi')
        self.n_round = self.get_parameter('merge_rounds').value
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)

        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cb_single, 10)

        # 缓存：Open3D 对象
        self.acc = o3d.geometry.PointCloud()
        self.cnt = 0

    # ---------------- 每来一圈 ----------------
    def cb_single(self, msg: PointCloud2):
        # 单圈 → Open3D
        pcd = self.ros2_to_open3d(msg)
        if not pcd.has_points():
            return

        # 累加
        self.acc += pcd          # Open3D 重载了 += （合并点云）
        self.cnt += 1

        # 达到 N 圈
        if self.cnt >= self.n_round:
            # 发布
            out_msg = self.open3d_to_ros2(self.acc, msg.header)
            self.pub.publish(out_msg)
            # self.get_logger().info(f'Published full cloud: {len(self.acc.points)} pts ({self.n_round} rounds)')

            # 清空缓存
            self.acc.clear()
            self.cnt = 0

    # ---------------- 工具：ROS ↔ Open3D ----------------
    def ros2_to_open3d(self, msg: PointCloud2):
        offset = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, step)

        xyz = np.stack([
            np.frombuffer(data[:, offset['x']: offset['x'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['y']: offset['y'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['z']: offset['z'] + 4].copy(), dtype=np.float32)
        ], axis=1)

        rgb = None
        if 'rgb' in offset:
            rgb_packed = np.frombuffer(
                data[:, offset['rgb']: offset['rgb'] + 4].copy(), dtype=np.uint32)
            r = (rgb_packed >> 16) & 0xFF
            g = (rgb_packed >> 8)  & 0xFF
            b =  rgb_packed        & 0xFF
            rgb = np.stack([r, g, b], axis=1) / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if rgb is not None:
            pcd.colors = o3d.utility.Vector3dVector(rgb)
        return pcd

    def open3d_to_ros2(self, pcd, header):
        pts = np.asarray(pcd.points)

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = header.frame_id
        cloud_msg.height = 1
        cloud_msg.width = pts.shape[0]
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False

        # 字段：xyz + intensity + ring + time  （与雷达完全一致）
        cloud_msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
            PointField(name='time',      offset=24, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 28          # 7*4 = 28
        cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width

        # 拼字节：xyz(12) + 预留(4) + intensity(4) + ring(2) + time(4) = 28
        buf = []
        for i, pt in enumerate(pts):
            buf.append(struct.pack('<fff', *pt))                    # xyz  0-11
            buf.append(b'\x00\x00\x00\x00')                         # pad  12-15
            # intensity 统一填 0，ring 填 0，time 填 0
            buf.append(struct.pack('<f', 0.0))                      # intensity 16-19
            buf.append(struct.pack('<H', 0))                        # ring     20-21
            buf.append(struct.pack('<f', 0.0))                      # time     22-25
            buf.append(b'\x00\x00')                                 # pad      26-27
        cloud_msg.data = b''.join(buf)
        return cloud_msg


def main():
    rclpy.init()
    node = MultiScanMerge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()