#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
import numpy as np
# import huateng_vision_sdk.mvsdk as mvsdk  # ← 你需要确保这个模块可用（见下方说明）
from .huateng_vision_sdk import mvsdk
from .huateng_vision_sdk.mvsdk import CameraException

# =================================================================
# 相机参数（可后续改为通过 ROS 参数配置）
# =================================================================
"""
相机分辨率：
0: 2048X1536 最大
1: 1920X1440 居中裁剪
2: 1600X1200 居中裁剪
3: 1440X1080 居中裁剪
4: 1280X960 居中裁剪
5: 1024X768 BIN2X2
6: 1024X768 居中裁剪
7: 1024X768 左上角
8: 1024X768 右上角
9: 1024X768 左下角
10: 1024X768 右下角
11: 512X384 SKIP4X4
12: 320X240 SKIP4X4
"""
LEFT_CAM_IDX = 0
RIGHT_CAM_IDX = 1
EXP_US = 15000
ANALOG_GAIN = 12
FRAME_SPEED = 2
TARGET_RESOLUTION_IDX = 11
SYNC_TH_MS = 100  # 同步阈值 (ms)



class IndustrialStereoDriver(Node):
    def __init__(self):
        super().__init__('industrial_stereo_driver')

        # Publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.bridge = CvBridge()

        # 相机资源
        self.hcamL = None
        self.hcamR = None
        self.bufL = None
        self.bufR = None
        self.running = False

        # 初始化相机
        self._init_cameras()

        # 启动采集线程
        self.running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('Industrial stereo camera driver started.')

    def _init_cameras(self):
        try:
            dev_list = mvsdk.CameraEnumerateDevice()
            if len(dev_list) < 2:
                raise RuntimeError("Need at least 2 cameras")

            # 初始化左右相机
            devL = dev_list[LEFT_CAM_IDX]
            devR = dev_list[RIGHT_CAM_IDX]
            self.hcamL = mvsdk.CameraInit(devL, -1, -1)
            self.hcamR = mvsdk.CameraInit(devR, -1, -1)

            # 获取能力并配置
            capL = mvsdk.CameraGetCapability(self.hcamL)
            capR = mvsdk.CameraGetCapability(self.hcamR)

            self._setup_camera(self.hcamL, capL)
            self._setup_camera(self.hcamR, capR)

            # 分配缓冲区
            buf_size = capL.sResolutionRange.iWidthMax * capL.sResolutionRange.iHeightMax * 3
            self.bufL = mvsdk.CameraAlignMalloc(buf_size, 16)
            self.bufR = mvsdk.CameraAlignMalloc(buf_size, 16)

            # 启动采集
            mvsdk.CameraPlay(self.hcamL)
            mvsdk.CameraPlay(self.hcamR)

        except Exception as e:
            self.get_logger().error(f'Camera init failed: {e}')
            raise

    def _setup_camera(self, hcam, cap):
        try:
            mvsdk.CameraSetIspOutFormat(hcam, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        except:
            self.get_logger().warn("Cannot set BGR8, will use software conversion")
        mvsdk.CameraSetTriggerMode(hcam, 0)
        mvsdk.CameraSetAeState(hcam, 0)
        mvsdk.CameraSetExposureTime(hcam, EXP_US)
        mvsdk.CameraSetAnalogGainX(hcam, ANALOG_GAIN)
        mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)
        if TARGET_RESOLUTION_IDX < cap.iImageSizeDesc:
            res_desc = cap.pImageSizeDesc[TARGET_RESOLUTION_IDX]
            mvsdk.CameraSetImageResolution(hcam, res_desc)

    def _grab_frame(self, hcam, buf):
        try:
            raw, head = mvsdk.CameraGetImageBuffer(hcam, 200)
            t = time.time()
            mvsdk.CameraImageProcess(hcam, raw, buf, head)
            mvsdk.CameraReleaseImageBuffer(hcam, raw)

            arr = np.frombuffer(
                (mvsdk.c_ubyte * head.uBytes).from_address(buf),
                dtype=np.uint8
            )
            frame = arr.reshape((head.iHeight, head.iWidth, -1))
            if len(frame.shape) == 2:
                frame = np.dstack([frame] * 3)
            elif frame.shape[2] == 1:
                frame = np.dstack([frame[:, :, 0]] * 3)
            return frame, t
        except CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().error(f'Grab frame error: {e.message}')
            return None, None

    def _capture_loop(self):
        while self.running:
            frameL, tL = self._grab_frame(self.hcamL, self.bufL)
            frameR, tR = self._grab_frame(self.hcamR, self.bufR)

            if frameL is None or frameR is None:
                continue

            # 可选：同步检查（如果不需要严格同步，可注释掉）
            if abs(tL - tR) > SYNC_TH_MS * 0.001:
                continue

            # 转换为 ROS Image 消息
            try:
                msgL = self.bridge.cv2_to_imgmsg(frameL, "bgr8")
                msgR = self.bridge.cv2_to_imgmsg(frameR, "bgr8")
                msgL.header.stamp = self.get_clock().now().to_msg()
                msgR.header.stamp = self.get_clock().now().to_msg()
                msgL.header.frame_id = "camera_left"
                msgR.header.frame_id = "camera_right"

                self.left_pub.publish(msgL)
                self.right_pub.publish(msgR)
            except Exception as e:
                self.get_logger().error(f'CV bridge error: {e}')

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        self._cleanup()
        super().destroy_node()

    def _cleanup(self):
        if self.hcamL:
            mvsdk.CameraUnInit(self.hcamL)
        if self.hcamR:
            mvsdk.CameraUnInit(self.hcamR)
        if self.bufL:
            mvsdk.CameraAlignFree(self.bufL)
        if self.bufR:
            mvsdk.CameraAlignFree(self.bufR)


def main(args=None):
    rclpy.init(args=args)
    node = IndustrialStereoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()