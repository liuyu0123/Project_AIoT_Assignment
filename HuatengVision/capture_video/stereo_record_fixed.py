#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双目同步录制 —— 帧率/时长修正版
"""
import cv2, time, threading, queue, os, datetime

LEFT_ID, RIGHT_ID = 3, 2          # << 改成你的实际编号
WIDTH, HEIGHT, FPS = 1280, 720, 30
SYNC_TH = 0.05                    # 50 ms 内都算同步（原 15 ms 太严）
OUT_DIR = "stereo_videos"

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
    cap.set(cv2.CAP_PROP_EXPOSURE, -7)   # 再往下降，减少拖影
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

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(OUT_DIR, f"stereo_{ts}.mp4")
    writer = None
    recording = False
    write_cnt = 0

    while True:
        try:
            tL, tR = lq.get(timeout=0.2), rq.get(timeout=0.2)
        except queue.Empty:
            continue
        # if abs(tL - tR) > SYNC_TH:          # 丢不同步帧
        #     continue
        ret1, imgL = lcap.retrieve()
        ret2, imgR = rcap.retrieve()
        if not (ret1 and ret2):
            continue

        frame = cv2.hconcat([imgL, imgR])   # 2560×720
        cv2.imshow('stereo (ESC退出, 空格录制/暂停)', cv2.resize(frame, (1280, 360)))

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            recording = not recording
            if recording and writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(out_path, fourcc, FPS, (w, h))
                print(f'[+] 开始录制 -> {out_path}  分辨率 {w}x{h}')
            print('[pause]' if not recording else '[recording]')

        if recording and writer is not None:
            writer.write(frame)
            write_cnt += 1
            print(f'\r已写入 {write_cnt} 帧', end='')

    if writer is not None:
        writer.release()
        duration = write_cnt / FPS
        print(f'\n[+] 已保存视频：{out_path}  总帧数={write_cnt}  预计时长={duration:.2f} s')
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
    # 预热
    for _ in range(FPS):
        left.grab(); right.grab()
        time.sleep(0.02)
    print('按空格开始/暂停录制，ESC 退出并生成视频')
    sync_record(left, right)
    left.release(); right.release()

if __name__ == '__main__':
    main()