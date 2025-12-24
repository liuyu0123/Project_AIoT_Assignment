#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2, time, threading, queue, os, datetime

LEFT_ID, RIGHT_ID = 3, 2
WIDTH, HEIGHT, FPS = 1280, 720, 30
OUT_DIR = "stereo_videos"

# ---------- 全局计数 ----------
g_lgrab = g_rgrab = g_lret = g_rret = g_write = 0
lock = threading.Lock()

def add_Lgrab(): global g_lgrab; lock.acquire(); g_lgrab += 1; lock.release()
def add_Rgrab(): global g_rgrab; lock.acquire(); g_rgrab += 1; lock.release()
def add_Lret():  global g_lret;  lock.acquire(); g_lret  += 1; lock.release()
def add_Rret():  global g_rret;  lock.acquire(); g_rret  += 1; lock.release()
def add_write(): global g_write; lock.acquire(); g_write += 1; lock.release()

def print_stats():
    global g_lgrab, g_rgrab, g_lret, g_rret, g_write
    while True:
        time.sleep(1)
        with lock:
            print(f'GRAB  L={g_lgrab} R={g_rgrab} | RETRIEVE  L={g_lret} R={g_rret} | WRITE={g_write}')
            g_lgrab, g_rgrab, g_lret, g_rret, g_write = 0, 0, 0, 0, 0

# ---------- 通用 ----------
def list_cameras(max_id=10):
    ids = []
    for i in range(max_id):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]: ids.append(i)
        cap.release()
    return ids

def open_camera(cid):
    for back in [cv2.CAP_MSMF, cv2.CAP_DSHOW]:
        cap = cv2.VideoCapture(cid, back)
        if cap.isOpened(): break
    else: raise RuntimeError(f'相机 {cid} 打不开')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -8)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    return cap

# ---------- 线程 ----------
def grab_thread(cap, q, is_left):
    while cap.isOpened():
        if cap.grab():
            q.put(time.time())
            (add_Lgrab if is_left else add_Rgrab)()

# ---------- 主循环 ----------
def sync_record(lcap, rcap):
    os.makedirs(OUT_DIR, exist_ok=True)
    lq, rq = queue.Queue(maxsize=5), queue.Queue(maxsize=5)
    threading.Thread(target=grab_thread, args=(lcap, lq, True), daemon=True).start()
    threading.Thread(target=grab_thread, args=(rcap, rq, False), daemon=True).start()
    threading.Thread(target=print_stats, daemon=True).start()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_path = os.path.join(OUT_DIR, f'stereo_{ts}.mp4')
    writer = None
    recording = False

    while True:
        try:
            tL, tR = lq.get(timeout=0.2), rq.get(timeout=0.2)
        except queue.Empty:
            continue
        ret1, imgL = lcap.retrieve()
        ret2, imgR = rcap.retrieve()
        if ret1: add_Lret()
        if ret2: add_Rret()
        if not (ret1 and ret2):
            continue

        frame = cv2.hconcat([imgL, imgR])
        cv2.imshow('debug', cv2.resize(frame, (1280, 360)))
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        if key == ord(' '):
            recording = not recording
            if recording and writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(out_path, fourcc, FPS, (w, h))
                print(f'[>] 录制 -> {out_path}  {w}x{h}')
        if recording and writer is not None:
            writer.write(frame)
            add_write()

    if writer:
        writer.release()
        print(f'[v] 保存  {out_path}')
    cv2.destroyAllWindows()

# ---------- main ----------
def main():
    cams = list_cameras()
    print('可用相机:', cams)
    if len(cams) < 2:
        raise RuntimeError('不足 2 台')
    left, right = open_camera(LEFT_ID), open_camera(RIGHT_ID)
    print(f'使用 LEFT={LEFT_ID}  RIGHT={RIGHT_ID}')
    for _ in range(FPS):
        left.grab()
        right.grab()
        time.sleep(0.02)
    print('空格录制/暂停，ESC退出')
    sync_record(left, right)
    left.release()
    right.release()

if __name__ == '__main__':
    main()