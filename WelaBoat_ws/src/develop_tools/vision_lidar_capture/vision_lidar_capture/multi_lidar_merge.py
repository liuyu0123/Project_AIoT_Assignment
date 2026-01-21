#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct


class MultiScanMerge(Node):
    def __init__(self):
        super().__init__('multi_scan_merge')

        # ---------------- 参数 ----------------
        self.declare_parameter('merge_rounds', 5)
        self.declare_parameter('output_topic', '/unilidar/cloud_multi')
        self.n_round = self.get_parameter('merge_rounds').value
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 10)

        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.cb_single, 10)

        # ---------------- 缓存全部字段 ----------------
        self.acc_pts   = []          # [(x,y,z), ...]
        self.acc_int   = []          # [intensity, ...]
        self.acc_ring  = []          # [ring, ...]
        self.acc_time  = []          # [time, ...]
        self.cnt       = 0

    # ---------------- 每来一圈 ----------------
    def cb_single(self, msg: PointCloud2):
        # 1. 字段偏移
        offset = {f.name: f.offset for f in msg.fields}
        step   = msg.point_step
        data   = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, step)

        # 2. 提取全部字段
        pts = np.stack([
            np.frombuffer(data[:, offset['x']: offset['x'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['y']: offset['y'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['z']: offset['z'] + 4].copy(), dtype=np.float32)
        ], axis=1)
        intensity = np.frombuffer(data[:, offset['intensity']: offset['intensity'] + 4].copy(), dtype=np.float32)
        ring      = np.frombuffer(data[:, offset['ring']: offset['ring'] + 2].copy(),   dtype=np.uint16)
        time      = np.frombuffer(data[:, offset['time']: offset['time'] + 4].copy(),   dtype=np.float32)

        # 3. 累加
        self.acc_pts.extend(pts)
        self.acc_int.extend(intensity)
        self.acc_ring.extend(ring)
        self.acc_time.extend(time)
        self.cnt += 1

        # 4. 达到 N 圈 → 发布
        if self.cnt >= self.n_round:
            cloud_msg = self.package_cloud(msg.header)
            self.pub.publish(cloud_msg)
            # self.get_logger().info(f'Published full cloud: {len(self.acc_pts)} pts ({self.n_round} rounds)')
            # 5. 清空
            self.acc_pts.clear(); self.acc_int.clear()
            self.acc_ring.clear(); self.acc_time.clear()
            self.cnt = 0

    # ---------------- 一字节不差拼帧 ----------------
    def package_cloud(self, header):
        pts  = np.asarray(self.acc_pts,  dtype=np.float32)
        ints = np.asarray(self.acc_int,  dtype=np.float32)
        rings= np.asarray(self.acc_ring, dtype=np.uint16)
        times= np.asarray(self.acc_time, dtype=np.float32)

        cloud_msg = PointCloud2()
        cloud_msg.header       = header
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        cloud_msg.header.frame_id = header.frame_id
        cloud_msg.height       = 1
        cloud_msg.width        = pts.shape[0]
        cloud_msg.is_dense     = True
        cloud_msg.is_bigendian = False

        # 字段与宇树原消息 100% 一致
        cloud_msg.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
            PointField(name='time',      offset=24, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 28
        cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width

        # 逐点 28 字节
        buf = []
        for i in range(pts.shape[0]):
            buf.append(struct.pack('<fff', pts[i][0], pts[i][1], pts[i][2]))  # 0-11
            buf.append(struct.pack('<f',  ints[i]))                           # 12-15
            buf.append(struct.pack('<H',  rings[i]))                          # 16-17
            buf.append(struct.pack('<f',  times[i]))                          # 18-21
            buf.append(b'\x00\x00\x00\x00')                                   # 22-25 pad
            buf.append(b'\x00\x00')                                           # 26-27 pad
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