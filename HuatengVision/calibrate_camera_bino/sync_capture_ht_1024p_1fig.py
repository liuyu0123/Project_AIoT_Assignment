#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 工业双目标定拍照脚本（拼接版）
    光圈/曝光/模拟增益/分辨率 四项已锁死，标定期间严禁再动！
"""
import mvsdk
import numpy as np
import cv2
import os
import datetime
import platform

LEFT_CAM_IDX = 1
RIGHT_CAM_IDX = 0

# >>> 以下四项一旦标定完成，严禁再动 <<<
EXP_US      = 1500         # 30 ms 曝光
ANALOG_GAIN = 12            # 12 dB 模拟增益
FRAME_SPEED = 2             # 0=低速 1=普通 2=高速
APERTURE_F  = 2.0           # 镜头光圈环刻度（仅供记录，物理锁死）
# >>> 以上四项一旦标定完成，严禁再动 <<<

OUT_DIR     = "calib"
SYNC_TH_MS  = 10            # 时间戳差 < 10 ms 认为同步
RES_W, RES_H = 1024, 768    # 目标分辨率

def sdk_open_calib_camera(dev_idx):
    devs = mvsdk.CameraEnumerateDevice()
    if dev_idx >= len(devs):
        raise RuntimeError(f"没有索引 {dev_idx} 的相机")
    dev = devs[dev_idx]
    hcam = mvsdk.CameraInit(dev, -1, -1)
    cap = mvsdk.CameraGetCapability(hcam)

    print("相机支持分辨率：")
    for i in range(cap.iImageSizeDesc):
        desc = cap.pImageSizeDesc[i]
        print(i, desc.iWidth, desc.iHeight, desc.GetDescription())

    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    # 连续采集 + 手动曝光
    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)
    mvsdk.CameraSetAnalogGainX(hcam, ANALOG_GAIN)
    mvsdk.CameraSetFrameSpeed(hcam, FRAME_SPEED)

    # 选 1024×768（索引 5，可按实际打印调整）
    target_idx = 5
    res_target = cap.pImageSizeDesc[target_idx]
    mvsdk.CameraSetImageResolution(hcam, res_target)

    mvsdk.CameraPlay(hcam)
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)
    return hcam, frame_buf, (RES_W, RES_H)

def grab_frame(hcam, buf, shape):
    try:
        raw, head = mvsdk.CameraGetImageBuffer(hcam, 200)
        t = datetime.datetime.now()
        mvsdk.CameraImageProcess(hcam, raw, buf, head)
        mvsdk.CameraReleaseImageBuffer(hcam, raw)
        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(buf, head, 1)
        arr = np.frombuffer((mvsdk.c_ubyte * head.uBytes).from_address(buf), dtype=np.uint8)
        frame = arr.reshape((head.iHeight, head.iWidth, -1))
        return frame, t
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print("grab error:", e.message)
        return None, None

def save_stereo_image(left, right, tL, tR, idx):
    os.makedirs(OUT_DIR, exist_ok=True)
    # 横向拼接
    stereo = np.hstack([left, right])
    # 文件名带毫秒时间戳，防止覆盖
    ts_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    fname = f"sync_{ts_str}.png"
    fpath = os.path.join(OUT_DIR, fname)
    cv2.imwrite(fpath, stereo)

    # 记录参数
    with open(f"{OUT_DIR}/calib_info.txt", "a", encoding="utf-8") as f:
        f.write(f"{fname}  "
                f"aperture=F{APERTURE_F}  "
                f"exp={EXP_US}us  "
                f"gain={ANALOG_GAIN}dB  "
                f"res={RES_W}x{RES_H}  "
                f"speed={FRAME_SPEED}  "
                f"sync-diff={abs((tL - tR).total_seconds()) * 1000:.1f} ms\n")
    return fname

def main():
    left_cam, left_buf, shape = sdk_open_calib_camera(LEFT_CAM_IDX)
    right_cam, right_buf, _   = sdk_open_calib_camera(RIGHT_CAM_IDX)
    print("左右工业相机已打开，参数已锁定用于标定：")
    print(f"  光圈刻度 = F{APERTURE_F}（物理锁死）")
    print(f"  曝光时间 = {EXP_US} us")
    print(f"  模拟增益 = {ANALOG_GAIN} dB")
    print(f"  分辨率   = {RES_W}x{RES_H}")
    print("【警告】标定期间严禁转动光圈/变焦/对焦环！")
    print("按 空格 拍照，ESC 结束并生成文件列表")

    count = 0
    while True:
        frmL, tL = grab_frame(left_cam, left_buf, shape)
        frmR, tR = grab_frame(right_cam, right_buf, shape)
        if frmL is None or frmR is None:
            continue
        # 简单同步
        if abs((tL - tR).total_seconds()) > SYNC_TH_MS * 0.001:
            continue

        preview = cv2.hconcat([frmL, frmR])
        cv2.imshow("calib_preview (ESC--quit, Space--capture)",
                   cv2.resize(preview, (int(RES_W * 1.5), int(RES_H // 2 * 1.5))))
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            fname = save_stereo_image(frmL, frmR, tL, tR, count)
            print(f"saved {fname}  sync-diff={abs((tL - tR).total_seconds()) * 1000:.1f} ms")
            count += 1

    # 生成文件列表（仅列出本次生成的 sync_*.png）
    with open(f"{OUT_DIR}/calib_list.txt", "w") as f:
        for fn in sorted(os.listdir(OUT_DIR)):
            if fn.startswith("sync_") and fn.endswith(".png"):
                f.write(fn + "\n")
    print(f"[v] 已保存 {count} 组拼接标定图片，列表见 {OUT_DIR}/calib_list.txt")

    # 清理
    mvsdk.CameraUnInit(left_cam)
    mvsdk.CameraUnInit(right_cam)
    mvsdk.CameraAlignFree(left_buf)
    mvsdk.CameraAlignFree(right_buf)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()