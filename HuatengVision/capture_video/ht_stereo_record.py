#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双目同步录制 —— 原生 SDK 版（可变帧率，无动态模糊）
"""
import cv2
import numpy as np
import mvsdk
import datetime
import time
import os
import threading
import queue
import platform

OUT_DIR = "stereo_videos"
EXP_US = 30000          # 1 ms 曝光
GAIN_DB = 12            # 6 dB 增益
SYNC_TH_MS = 10        # 时间戳差 < 10 ms 即认为同步
FRAME_SPEED = 0        # 0=低速 1=普通 2=高速（120 fps）

# -------------------------------------------------
def sdk_open_camera(dev_idx):
    dev_list = mvsdk.CameraEnumerateDevice()
    if dev_idx >= len(dev_list):
        raise RuntimeError(f"没有索引为 {dev_idx} 的相机")
    dev = dev_list[dev_idx]
    hcam = mvsdk.CameraInit(dev, -1, -1)
    cap = mvsdk.CameraGetCapability(hcam)

    # 彩色/黑白
    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    # 连续采集 + 手动曝光
    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)      # 单位 us
    mvsdk.CameraSetAnalogGainX(hcam, GAIN_DB)
    mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)

    # 分辨率选 1280×720（索引 1 通常是 1280×720，若不对请按 PrintCap 修改）
    res_list = [cap.pImageSizeDesc[i] for i in range(cap.iImageSizeDesc)]
    idx_720p = next((i for i, r in enumerate(res_list) if r.iWidth == 1280 and r.iHeight == 720), 0)
    mvsdk.CameraSetImageResolution(hcam, res_list[idx_720p])

    mvsdk.CameraPlay(hcam)

    # 分配 buffer
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)
    return hcam, frame_buf, (1280, 720)


def sdk_grab_frame(hcam, buf, shape):
    try:
        raw, head = mvsdk.CameraGetImageBuffer(hcam, 200)
        t = time.time()
        mvsdk.CameraImageProcess(hcam, raw, buf, head)
        mvsdk.CameraReleaseImageBuffer(hcam, raw)
        # Windows 需要翻转
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(buf, head, 1)
        arr = np.frombuffer((mvsdk.c_ubyte * head.uBytes).from_address(buf), dtype=np.uint8)
        frame = arr.reshape((head.iHeight, head.iWidth, -1))
        return frame, t
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("grab error:", e.message)
        return None, 0


# -------------------------------------------------
def sync_record(left_idx, right_idx):
    os.makedirs(OUT_DIR, exist_ok=True)
    hcamL, bufL, shape = sdk_open_camera(left_idx)
    hcamR, bufR, _ = sdk_open_camera(right_idx)
    print("左右相机已打开，曝光={} us，增益={} dB，帧速档={}".format(EXP_US, GAIN_DB, FRAME_SPEED))

    frames = []          # [(frame_L, frame_R, t_L, t_R), ...]
    recording = False
    start_t = None

    while True:
        frmL, tL = sdk_grab_frame(hcamL, bufL, shape)
        frmR, tR = sdk_grab_frame(hcamR, bufR, shape)
        if frmL is None or frmR is None:
            continue
        # 简单同步：时间差 < 10 ms
        if abs(tL - tR) > SYNC_TH_MS * 0.001:
            continue

        # 预览（左右拼接）
        preview = cv2.hconcat([frmL, frmR])
        cv2.imshow("stereo (ESC退出, 空格录制/暂停)", cv2.resize(preview, (0, 0), fx=0.25, fy=0.25))

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            recording = not recording
            if recording and start_t is None:
                start_t = time.time()
                print('[>] 开始录制...')
            print('[pause]' if not recording else '[recording]')

        if recording:
            frames.append((frmL.copy(), frmR.copy(), tL, tR))

    # 写视频
    if frames:
        avg_fps = len(frames) / (frames[-1][2] - frames[0][2])
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(OUT_DIR, f"stereo_vfr_{ts}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frames[0][0].shape[:2]
        writer = cv2.VideoWriter(out_path, fourcc, avg_fps, (w * 2, h))
        for frmL, frmR, _, _ in frames:
            writer.write(cv2.hconcat([frmL, frmR]))
        writer.release()
        print(f'[v] 已保存  {out_path}')
        print(f'    总帧数={len(frames)}  平均fps={avg_fps:.2f}  时长={len(frames)/avg_fps:.2f}s')

    # 清理
    mvsdk.CameraUnInit(hcamL)
    mvsdk.CameraUnInit(hcamR)
    mvsdk.CameraAlignFree(bufL)
    mvsdk.CameraAlignFree(bufR)
    cv2.destroyAllWindows()


# -------------------------------------------------
def main():
    devs = mvsdk.CameraEnumerateDevice()
    if len(devs) < 2:
        raise RuntimeError("至少需要 2 台 HT-UBS300C-T")
    for i, d in enumerate(devs):
        print(f"{i}: {d.GetFriendlyName()}  {d.GetPortType()}")
    # 默认用 0 1，如需改手动输入
    left_idx = 1
    right_idx = 0
    sync_record(left_idx, right_idx)


if __name__ == '__main__':
    main()