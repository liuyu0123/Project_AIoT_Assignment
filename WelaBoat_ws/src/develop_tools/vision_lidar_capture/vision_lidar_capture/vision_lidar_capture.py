#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import open3d as o3d
import os
import threading
import sys
import termios
import tty
from ament_index_python.packages import get_package_share_directory
import select


class SyncCapture(Node):
    def __init__(self):
        super().__init__('sync_capture')

        # 1. 保存根目录 <ws>/data
        ws_root = self.find_workspace_root()
        self.data_root = os.path.join(ws_root, 'data')
        self.sub_dirs = {
            'left':  os.path.join(self.data_root, 'left'),
            'right': os.path.join(self.data_root, 'right'),
            'lidar': os.path.join(self.data_root, 'lidar')
        }
        for d in self.sub_dirs.values():
            os.makedirs(d, exist_ok=True)

        # 2. 序号：永远接最大已有+1
        self.seq = self.find_max_seq() + 1
        self.get_logger().info(f'Starting seq = {self.seq:04d}, root = {self.data_root}')

        # 3. 缓存最新数据
        self.br = CvBridge()
        self.latest = {'left': None, 'right': None, 'lidar': None}

        # 4. 订阅
        self.create_subscription(Image,      '/camera/left/image_raw',  self.cb_img_left,  1)
        self.create_subscription(Image,      '/camera/right/image_raw', self.cb_img_right, 1)
        self.create_subscription(PointCloud2,'/unilidar/cloud',         self.cb_lidar,     1)

        # 5. 键盘线程
        threading.Thread(target=self.keyboard_thread, daemon=True).start()

    # ---------------- 工具 ----------------
    def find_workspace_root(self):
        pkg_dir = get_package_share_directory(
            __name__.split('.')[0])   # vision_lidar_capture
        p = pkg_dir
        while p != '/':
            if os.path.isdir(os.path.join(p, 'src')):
                return p
            p = os.path.dirname(p)
        return os.path.expanduser('~')

    def find_max_seq(self):
        max_n = -1
        for folder in self.sub_dirs.values():
            if not os.path.isdir(folder):
                continue
            for f in os.listdir(folder):
                stem, _ = os.path.splitext(f)
                if stem.isdigit():
                    max_n = max(max_n, int(stem))
        return max_n

    # ---------------- 回调 ----------------
    def cb_img_left(self, msg):  self.latest['left']  = msg
    def cb_img_right(self, msg): self.latest['right'] = msg
    def cb_lidar(self, msg):     self.latest['lidar'] = msg

    # ---------------- 键盘 ----------------
    def keyboard_thread(self):
        while rclpy.ok():
            ch = self.getchar(timeout=0.2)   # 非阻塞，超时200ms
            if ch is None:                   # 超时，继续循环
                continue
            if ch == 's':
                self.save_once()
            elif ch == 'q':
                self.get_logger().info('User quit.')
                rclpy.shutdown()
                break

    # @staticmethod
    # def getchar(self, timeout=0.2):
    #     """非阻塞读字符；超时返回 None"""
    #     fd = sys.stdin.fileno()
    #     old = termios.tcgetattr(fd)
    #     ch = None
    #     try:
    #         tty.setraw(fd)
    #         # 用 select 给 stdin 设超时
    #         if select.select([sys.stdin], [], [], timeout)[0]:
    #             ch = sys.stdin.read(1)
    #     finally:
    #         termios.tcsetattr(fd, termios.TCSADRAIN, old)
    #     return ch

    def getchar(self, timeout=0.2):
        """非阻塞读字符；超时返回 None"""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        ch = None
        try:
            tty.setraw(fd)
            if select.select([sys.stdin], [], [], timeout)[0]:
                ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    # ---------------- 保存 ----------------
    def save_once(self):
        if any(v is None for v in self.latest.values()):
            self.get_logger().warn('Data not ready, skip.')
            return

        seq = self.seq
        self.seq += 1

        # 1. 左右图
        for side in ('left', 'right'):
            cv_img = self.br.imgmsg_to_cv2(self.latest[side], 'bgr8')
            path = os.path.join(self.sub_dirs[side], f'{seq:04d}.png')
            cv2.imwrite(path, cv_img)
            self.get_logger().info(f'Saved {path}')

        # 2. 点云
        pcd = self.pointcloud2_to_open3d(self.latest['lidar'])
        path = os.path.join(self.sub_dirs['lidar'], f'{seq:04d}.pcd')
        o3d.io.write_point_cloud(path, pcd)
        self.get_logger().info(f'Saved {path}')

    # ---------------- 新的 Open3D 接口 ----------------
    def pointcloud2_to_open3d(self, cloud_msg: PointCloud2):
        offset = {f.name: f.offset for f in cloud_msg.fields}
        step   = cloud_msg.point_step
        data   = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(-1, step)

        # **** 关键：先 .copy() 保证 C-contiguous ****
        xyz = np.stack([
            np.frombuffer(data[:, offset['x']: offset['x'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['y']: offset['y'] + 4].copy(), dtype=np.float32),
            np.frombuffer(data[:, offset['z']: offset['z'] + 4].copy(), dtype=np.float32)
        ], axis=1)

        rgb = None
        if 'rgb' in offset:
            rgb_packed = np.frombuffer(data[:, offset['rgb']: offset['rgb'] + 4].copy(),
                                       dtype=np.uint32)
            r = (rgb_packed >> 16) & 0xFF
            g = (rgb_packed >> 8)  & 0xFF
            b =  rgb_packed        & 0xFF
            rgb = np.stack([r, g, b], axis=1) / 255.0

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if rgb is not None:
            pcd.colors = o3d.utility.Vector3dVector(rgb)
        return pcd


def main():
    rclpy.init()
    node = SyncCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()