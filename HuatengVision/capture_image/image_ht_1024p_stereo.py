#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双相机同步一次性拍照 + 横向拼接保存
（无预览，运行一次拍一张拼接图，立即退出）
"""
import mvsdk
import numpy as np
import cv2
import os
import datetime
import platform

LEFT_CAM_IDX = 1
RIGHT_CAM_IDX = 0
# >>> 以下参数与标定脚本保持一致，按需微调 <<<
EXP_US      = 5000         # 30 ms 曝光
ANALOG_GAIN = 12            # 12 dB 模拟增益
FRAME_SPEED = 2             # 0=低速 1=普通 2=高速
RES_W, RES_H = 1024, 768    # 目标分辨率
OUT_DIR     = "stereo_snap" # 保存目录
SYNC_TH_MS  = 10            # 时间戳差 < 10 ms 认为同步
# >>> 以上参数与标定脚本保持一致，按需微调 <<<

def open_camera(dev_idx):
    """打开指定索引相机，返回句柄、缓冲区、shape"""
    devs = mvsdk.CameraEnumerateDevice()
    if dev_idx >= len(devs):
        raise RuntimeError(f"没有索引 {dev_idx} 的相机")
    dev = devs[dev_idx]
    hcam = mvsdk.CameraInit(dev, -1, -1)
    cap = mvsdk.CameraGetCapability(hcam)

    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)
    mvsdk.CameraSetAnalogGainX(hcam, ANALOG_GAIN)
    mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)

    target_idx = 5  # 1024×768 在我机器上索引为 5，不对请打印后改
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
    hcamL, bufL, _ = open_camera(LEFT_CAM_IDX)
    hcamR, bufR, _ = open_camera(RIGHT_CAM_IDX)

    print("正在同步拍照 …")
    # 简单同步：连续抓直到时间戳差 < 阈值
    while True:
        imgL, tL = grab_one_frame(hcamL, bufL)
        imgR, tR = grab_one_frame(hcamR, bufR)
        if imgL is None or imgR is None:
            continue
        if abs((tL - tR).total_seconds()) * 1000 <= SYNC_TH_MS:
            break

    # 横向拼接
    stitched = cv2.hconcat([imgL, imgR])
    fname = f"{OUT_DIR}/{tL.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.png"
    cv2.imwrite(fname, stitched)
    print(f"已保存拼接图：{fname}  分辨率={stitched.shape[1]}x{stitched.shape[0]}")

    # 清理
    for h, b in ((hcamL, bufL), (hcamR, bufR)):
        mvsdk.CameraUnInit(h)
        mvsdk.CameraAlignFree(b)

if __name__ == '__main__':
    main()