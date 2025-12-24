#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双目同步录制 —— 直接生成左右拼接视频
"""
import cv2, time, threading, queue, os, datetime

LEFT_ID, RIGHT_ID = 3, 2          # 根据你的枚举结果改
WIDTH, HEIGHT, FPS = 1280, 720, 30
OUT_DIR = "stereo_videos"         # 视频保存目录

# ---------- 通用函数 ----------
def list_cameras(max_id=10):
    ids = []
    for i in range(max_id):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            ids.append(i)
        cap.release()
    return ids

def open_camera(cid):
    for back in [cv2.CAP_MSMF, cv2.CAP_DSHOW]:
        cap = cv2.VideoCapture(cid, back)
        if cap.isOpened():
            break
    else:
        raise RuntimeError(f'相机 {cid} 无法打开')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    return cap

# ---------- 线程函数 ----------
def grab_thread(cap, q):
    while cap.isOpened():
        if cap.grab():
            q.put(time.time())
        else:
            time.sleep(0.001)

# ---------- 主录制循环 ----------
def sync_record(lcap, rcap):
    os.makedirs(OUT_DIR, exist_ok=True)
    lq, rq = queue.Queue(maxsize=5), queue.Queue(maxsize=5)
    threading.Thread(target=grab_thread, args=(lcap, lq), daemon=True).start()
    threading.Thread(target=grab_thread, args=(rcap, rq), daemon=True).start()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')          # H.264 可用 *'avc1'，若编码失败再换回 mp4v
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(OUT_DIR, f"stereo_{ts}.mp4")
    writer = None
    recording = False

    while True:
        try:
            tL, tR = lq.get(timeout=0.2), rq.get(timeout=0.2)
        except queue.Empty:
            continue
        if abs(tL - tR) > 0.015:          # 15 ms 外丢弃
            continue
        ret1, imgL = lcap.retrieve()
        ret2, imgR = rcap.retrieve()
        if not (ret1 and ret2):
            continue

        frame = cv2.hconcat([imgL, imgR])  # 左右拼接 → 2560×720
        cv2.imshow('stereo (ESC退出, 空格录制/暂停)', cv2.resize(frame, (1280, 360)))

        key = cv2.waitKey(1) & 0xFF
        if key == 27:                      # ESC 退出
            break
        if key == ord(' '):                # 空格 开始/暂停
            recording = not recording
            if recording and writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(out_path, fourcc, FPS, (w, h))
                print(f'[+] 开始录制 -> {out_path}')
            print('[pause]' if not recording else '[recording]')

        if recording and writer is not None:
            writer.write(frame)

    if writer is not None:
        writer.release()
        print(f'[+] 已保存视频：{out_path}')
    cv2.destroyAllWindows()

# ---------- 主入口 ----------
def main():
    print('枚举相机 …')
    cams = list_cameras()
    print('可用序号：', cams)
    if len(cams) < 2:
        raise RuntimeError('不足 2 台相机')
    left, right = open_camera(LEFT_ID), open_camera(RIGHT_ID)
    print(f'使用 LEFT={LEFT_ID}  RIGHT={RIGHT_ID}')
    # 预热 1 s
    for _ in range(FPS):
        left.grab(); right.grab()
        time.sleep(0.02)
    print('按空格开始/暂停录制，ESC 退出并生成视频')
    sync_record(left, right)
    left.release(); right.release()

if __name__ == '__main__':
    main()