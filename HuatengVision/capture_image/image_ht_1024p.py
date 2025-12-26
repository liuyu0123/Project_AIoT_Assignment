#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 工业相机一次性拍照脚本
（无预览，运行一次拍一张，立即保存并退出）
"""
import mvsdk
import numpy as np
import cv2
import os
import datetime
import platform

# >>> 以下参数与标定脚本保持一致，按需微调 <<<
EXP_US      = 30000         # 30 ms 曝光
ANALOG_GAIN = 12            # 12 dB 模拟增益
FRAME_SPEED = 2             # 0=低速 1=普通 2=高速
RES_W, RES_H = 1024, 768    # 目标分辨率
OUT_DIR     = "snap"        # 保存目录
# >>> 以上参数与标定脚本保持一致，按需微调 <<<

def open_camera(dev_idx=0):
    """打开指定索引的相机并返回句柄、缓冲区、shape"""
    devs = mvsdk.CameraEnumerateDevice()
    if dev_idx >= len(devs):
        raise RuntimeError(f"没有索引 {dev_idx} 的相机")
    dev = devs[dev_idx]
    hcam = mvsdk.CameraInit(dev, -1, -1)
    cap = mvsdk.CameraGetCapability(hcam)

    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    # 连续采集 + 手动曝光
    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)
    mvsdk.CameraSetAnalogGainX(hcam, ANALOG_GAIN)
    mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)

    # 选 1024×768（索引 5 是我机器上的，若不对请打印后自行修改）
    target_idx = 5
    res = cap.pImageSizeDesc[target_idx]
    mvsdk.CameraSetImageResolution(hcam, res)

    mvsdk.CameraPlay(hcam)
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)
    return hcam, frame_buf, (RES_W, RES_H)

def grab_one_frame(hcam, buf):
    """抓取一帧，成功返回 (image, timestamp)，失败返回 (None, None)"""
    try:
        raw, head = mvsdk.CameraGetImageBuffer(hcam, 1000)
        t = datetime.datetime.now()
        mvsdk.CameraImageProcess(hcam, raw, buf, head)
        mvsdk.CameraReleaseImageBuffer(hcam, raw)
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(buf, head, 1)
        arr = np.frombuffer((mvsdk.c_ubyte * head.uBytes).from_address(buf), dtype=np.uint8)
        img = arr.reshape((head.iHeight, head.iWidth, -1))
        return img, t
    except mvsdk.CameraException as e:
        print("grab error:", e.message)
        return None, None

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    hcam, buf, shape = open_camera(0)

    print("正在拍照 …")
    img, t = grab_one_frame(hcam, buf)
    if img is None:
        print("拍照失败！")
        return

    # 按时间戳命名，避免覆盖
    fname = f"{OUT_DIR}/{t.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.png"
    cv2.imwrite(fname, img)
    print(f"已保存：{fname}  分辨率={img.shape[1]}x{img.shape[0]}")

    # 清理
    mvsdk.CameraUnInit(hcam)
    mvsdk.CameraAlignFree(buf)

if __name__ == '__main__':
    main()