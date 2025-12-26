#coding=utf-8
import mvsdk
import numpy as np
import cv2
import platform

DEV_IDX   = 0            # 先用 0 号相机
EXP_US    = 10 * 3000   # 10 ms
GAIN_DB   = 12
SPEED     = 0

def main():
    devs = mvsdk.CameraEnumerateDevice()
    if DEV_IDX >= len(devs):
        raise RuntimeError(f"没有索引 {DEV_IDX} 的相机")
    print(f"打开 {devs[DEV_IDX].GetFriendlyName()}  {devs[DEV_IDX].GetPortType()}")

    hcam = mvsdk.CameraInit(devs[DEV_IDX], -1, -1)
    cap  = mvsdk.CameraGetCapability(hcam)
    mono = cap.sIspCapacity.bMonoSensor != 0
    fmt  = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hcam, fmt)

    mvsdk.CameraSetTriggerMode(hcam, 0)
    mvsdk.CameraSetAeState(hcam, 0)
    mvsdk.CameraSetExposureTime(hcam, EXP_US)
    mvsdk.CameraSetAnalogGainX(hcam, GAIN_DB)
    mvsdk.CameraSetFrameSpeed(hcam, SPEED)

    # 选 640×480
    res_480p = next((cap.pImageSizeDesc[i] for i in range(cap.iImageSizeDesc)
                     if cap.pImageSizeDesc[i].iWidth == 640 and cap.pImageSizeDesc[i].iHeight == 480),
                    cap.pImageSizeDesc[0])
    mvsdk.CameraSetImageResolution(hcam, res_480p)

    mvsdk.CameraPlay(hcam)

    # ✅ 按最大分辨率分配 buffer，而不是按当前 640×480
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)

    ok = timeout = 0
    print("开始取流……  'q' 退出")
    while True:
        try:
            raw, head = mvsdk.CameraGetImageBuffer(hcam, 200)
            ok += 1
            mvsdk.CameraImageProcess(hcam, raw, frame_buf, head)
            mvsdk.CameraReleaseImageBuffer(hcam, raw)
            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(frame_buf, head, 1)
            arr = np.frombuffer((mvsdk.c_ubyte * head.uBytes).from_address(frame_buf), np.uint8)
            img = arr.reshape((head.iHeight, head.iWidth, -1))
            cv2.imshow("single_test", cv2.resize(img, (640, 480)))
        except mvsdk.CameraException as e:
            if e.error_code == mvsdk.CAMERA_STATUS_TIME_OUT:
                timeout += 1
                print('.', end='')
            else:
                print("其他错误:", e.message)
            img = None

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print(f"\n抓取统计  成功:{ok}  超时:{timeout}")
    mvsdk.CameraUnInit(hcam)
    mvsdk.CameraAlignFree(frame_buf)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()