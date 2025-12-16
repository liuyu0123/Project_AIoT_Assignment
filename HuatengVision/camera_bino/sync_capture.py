#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HT-UBS300C-T 双 USB 相机同步采集 —— 队列防爆满版
"""
import cv2, time, threading, queue, os

LEFT_ID, RIGHT_ID = 3, 2          # 根据你的枚举结果改
WIDTH, HEIGHT, FPS = 1280, 720, 30

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
    """只把抓取成功的时间戳丢进队列，主线程负责 retrieve"""
    while cap.isOpened():
        if cap.grab():
            q.put(time.time())
        else:
            time.sleep(0.001)

# ---------- 主采集循环 ----------
def sync_capture(lcap, rcap, root='calib'):
    os.makedirs(f'{root}/left', exist_ok=True)
    os.makedirs(f'{root}/right', exist_ok=True)
    lq, rq = queue.Queue(maxsize=5), queue.Queue(maxsize=5)
    threading.Thread(target=grab_thread, args=(lcap, lq), daemon=True).start()
    threading.Thread(target=grab_thread, args=(rcap, rq), daemon=True).start()

    count = 0
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

        # 显示
        cv2.imshow('left',  cv2.resize(imgL, (WIDTH//2, HEIGHT//2)))
        cv2.imshow('right', cv2.resize(imgR, (WIDTH//2, HEIGHT//2)))
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            cv2.imwrite(f'{root}/left/{count:03d}.png',  imgL)
            cv2.imwrite(f'{root}/right/{count:03d}.png', imgR)
            print(f'saved {count:03d}  sync-diff={abs(tL-tR)*1000:.1f} ms')
            count += 1
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
    print('按空格拍照，ESC 退出')
    sync_capture(left, right)
    left.release(); right.release()

if __name__ == '__main__':
    main()