import cv2, threading, queue, time

# ---------- 参数 ----------
LEFT_ID  = 2
RIGHT_ID = 3
WIDTH    = 1280
HEIGHT   = 720
SHOW_W   = 640
SHOW_H   = 360
SYNC_TH  = 0.015

# ---------- 打开相机 ----------
def open_cam(cid):
    # 1. 只改这里：DSHOW 兼容 grab 线程
    cap = cv2.VideoCapture(cid, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

capL = open_cam(LEFT_ID)
capR = open_cam(RIGHT_ID)

# ---------- 线程 ----------
def grab_loop(cap, q):
    while cap.isOpened():
        if cap.grab():
            q.put(time.time())
        else:
            time.sleep(0.001)

qL, qR = queue.Queue(maxsize=3), queue.Queue(maxsize=3)
threading.Thread(target=grab_loop, args=(capL, qL), daemon=True).start()
threading.Thread(target=grab_loop, args=(capR, qR), daemon=True).start()




# ---------- 供外部脚本调用 ----------
def get_sync_frame(timeout=0.2):
    try:
        tL, tR = qL.get(timeout=timeout), qR.get(timeout=0.2)
    except queue.Empty:
        return None, None
    if abs(tL - tR) > SYNC_TH:
        return None, None
    ret1, imgL = capL.retrieve()
    ret2, imgR = capR.retrieve()
    return (imgL, imgR) if (ret1 and ret2) else (None, None)

IMG_SIZE = (WIDTH, HEIGHT)

if __name__ == '__main__':
    # ---------- 主循环 ----------
    print('按 ESC 退出')
    while True:
        try:
            tL, tR = qL.get(timeout=0.2), qR.get(timeout=0.2)
        except queue.Empty:
            continue
        if abs(tL - tR) > SYNC_TH:
            continue

        ret1, imgL = capL.retrieve()
        ret2, imgR = capR.retrieve()
        if not (ret1 and ret2):
            continue

        cv2.imshow('left',  cv2.resize(imgL, (SHOW_W, SHOW_H)))
        cv2.imshow('right', cv2.resize(imgR, (SHOW_W, SHOW_H)))
        if cv2.waitKey(1) & 0xFF == 27:
            break

    capL.release(); capR.release(); cv2.destroyAllWindows()