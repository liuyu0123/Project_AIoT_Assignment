#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 工业双目标定拍照脚本
    光圈/曝光/模拟增益/分辨率 四项已锁死，标定期间严禁再动！
"""
import mvsdk
import numpy as np
import cv2
import os
import datetime
import platform

# >>> 以下四项一旦标定完成，严禁再动 <<<
EXP_US      = 30000         # 30 ms 曝光
ANALOG_GAIN = 12            # 12 dB 模拟增益
FRAME_SPEED = 2             # 0=低速 1=普通 2=高速
APERTURE_F  = 2.0           # 镜头光圈环刻度（仅供记录，物理锁死）
# >>> 以上四项一旦标定完成，严禁再动 <<<

OUT_DIR     = "calib"
SYNC_TH_MS  = 10            # 时间戳差 < 10 ms 认为同步
RES_W, RES_H = 1024, 768    # 目标分辨率（索引 1 通常是 720p）

def sdk_open_calib_camera(dev_idx):
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

    # 选 1280×720
    res_720p = next((cap.pImageSizeDesc[i] for i in range(cap.iImageSizeDesc)
                     if cap.pImageSizeDesc[i].iWidth == RES_W and cap.pImageSizeDesc[i].iHeight == RES_H),
                    cap.pImageSizeDesc[0])
    mvsdk.CameraSetImageResolution(hcam, res_720p)

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


def save_calib_image(left, right, idx):
    os.makedirs(f"{OUT_DIR}/left", exist_ok=True)
    os.makedirs(f"{OUT_DIR}/right", exist_ok=True)
    cv2.imwrite(f"{OUT_DIR}/left/{idx:03d}.png", left)
    cv2.imwrite(f"{OUT_DIR}/right/{idx:03d}.png", right)
    # 记录参数
    with open(f"{OUT_DIR}/calib_info.txt", "a", encoding="utf-8") as f:
        f.write(f"{idx:03d}  "
                f"aperture=F{APERTURE_F}  "
                f"exp={EXP_US}us  "
                f"gain={ANALOG_GAIN}dB  "
                f"res={RES_W}x{RES_H}  "
                f"speed={FRAME_SPEED}\n")


def main():
    left_cam, left_buf, shape = sdk_open_calib_camera(0)
    right_cam, right_buf, _ = sdk_open_calib_camera(1)
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
        cv2.imshow("calib_preview (ESC退出, 空格拍照)", cv2.resize(preview, (RES_W // 2, RES_H // 2)))
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            save_calib_image(frmL, frmR, count)
            print(f"saved {count:03d}  sync-diff={abs((tL - tR).total_seconds()) * 1000:.1f} ms")
            count += 1

    # 生成文件列表
    with open(f"{OUT_DIR}/calib_list.txt", "w") as f:
        for i in range(count):
            f.write(f"left/{i:03d}.png  right/{i:03d}.png\n")
    print(f"[v] 已保存 {count} 组标定图片，列表见 {OUT_DIR}/calib_list.txt")

    # 清理
    mvsdk.CameraUnInit(left_cam)
    mvsdk.CameraUnInit(right_cam)
    mvsdk.CameraAlignFree(left_buf)
    mvsdk.CameraAlignFree(right_buf)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()