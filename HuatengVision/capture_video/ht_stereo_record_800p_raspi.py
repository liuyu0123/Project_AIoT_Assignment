#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双目同步录制 —— 无窗口后台版（按真实帧率写文件）
"""
import cv2
import numpy as np
import mvsdk
import datetime
import time
import os
import signal
import sys

# ---------- 参数 ----------
OUT_DIR      = "stereo_videos"
EXP_US       = 300          # 30 ms
GAIN_DB      = 12
FRAME_SPEED  = 2             # 2=120 fps
SYNC_TH_MS   = 100           # 同步阈值
LEFT_IDX     = 1
RIGHT_IDX    = 0
# --------------------------

stop_flag = False
def signal_handler(sig, frame):
    global stop_flag
    print("\n正在停止采集...")
    stop_flag = True
signal.signal(signal.SIGINT, signal_handler)

def sdk_open_camera(dev_idx):
    dev_list = mvsdk.CameraEnumerateDevice()
    if dev_idx >= len(dev_list):
        raise RuntimeError(f"没有索引为 {dev_idx} 的相机")
    dev = dev_list[dev_idx]
    hcam = mvsdk.CameraInit(dev, -1, -1)
    cap = mvsdk.CameraGetCapability(hcam)

    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)
    mvsdk.CameraSetAnalogGainX(hcam, GAIN_DB)
    mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)

    """
    ################### Linux ##################
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
    target_idx = 11
    res_720p = cap.pImageSizeDesc[target_idx]
    mvsdk.CameraSetImageResolution(hcam, res_720p)
    mvsdk.CameraPlay(hcam)

    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)
    return hcam, frame_buf, (800, 600)

def sdk_grab_frame(hcam, buf, shape):
    try:
        raw, head = mvsdk.CameraGetImageBuffer(hcam, 200)
        t = time.time()
        mvsdk.CameraImageProcess(hcam, raw, buf, head)
        mvsdk.CameraReleaseImageBuffer(hcam, raw)
        arr = np.frombuffer((mvsdk.c_ubyte * head.uBytes).from_address(buf), dtype=np.uint8)
        frame = arr.reshape((head.iHeight, head.iWidth, -1))
        return frame, t
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("grab error:", e.message)
        return None, 0

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    hcamL, bufL, shape = sdk_open_camera(LEFT_IDX)
    hcamR, bufR, _     = sdk_open_camera(RIGHT_IDX)
    print("左右相机已打开，按 Ctrl-C 停止录制")

    frames = []                      # [(frmL, frmR, t), ...]
    timestamp_start = datetime.datetime.now()
    while not stop_flag:
        frmL, tL = sdk_grab_frame(hcamL, bufL, shape)
        frmR, tR = sdk_grab_frame(hcamR, bufR, shape)
        if frmL is None or frmR is None:
            continue
        if abs(tL - tR) > SYNC_TH_MS * 0.001:
            continue
        frames.append((frmL.copy(), frmR.copy(), tL))   # 把左相机时间戳存下来
        timestamp_end = datetime.datetime.now()
        print(f"已经录像：{(timestamp_end - timestamp_start).total_seconds()} 秒")
        if (timestamp_end - timestamp_start).total_seconds() >= 20: #秒
            print("记录时间达到最大值，录像结束...")
            break

    print('\n采集环已退出，等待 SDK 释放流...')
    time.sleep(0.2)

    # 写视频
    if frames:
        duration = frames[-1][2] - frames[0][2]        # 实际采集时长
        avg_fps = len(frames) / duration if duration else 30
        print(f'实际时长 {duration:.2f}s，平均帧率 {avg_fps:.2f} fps')

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(OUT_DIR, f"stereo_{ts}_expose{EXP_US}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frames[0][0].shape[:2]
        writer = cv2.VideoWriter(out_path, fourcc, avg_fps, (w * 2, h))
        for frmL, frmR, _ in frames:
            writer.write(cv2.hconcat([frmL, frmR]))
        writer.release()
        print(f'已保存  {out_path}  总帧数={len(frames)}')

    # 清理
    mvsdk.CameraUnInit(hcamL)
    mvsdk.CameraUnInit(hcamR)
    mvsdk.CameraAlignFree(bufL)
    mvsdk.CameraAlignFree(bufR)
    print("相机已安全关闭，退出。")

if __name__ == '__main__':
    main()