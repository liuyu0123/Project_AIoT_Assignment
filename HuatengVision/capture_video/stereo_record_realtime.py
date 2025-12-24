#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目录制 —— 可变帧率（卡但不快放）
"""
import cv2, time, threading, queue, os, datetime

LEFT_ID, RIGHT_ID = 3, 2
WIDTH, HEIGHT = 1280, 720          # 不再强制 FPS
OUT_DIR = "stereo_videos"

# ---------- 工具 ----------
def list_cameras(max_id=10):
    ids=[]
    for i in range(max_id):
        cap=cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.read()[0]: ids.append(i)
        cap.release()
    return ids

def open_camera(cid):
    # for back in [cv2.CAP_MSMF, cv2.CAP_DSHOW]:
    for back in [cv2.CAP_MSMF]:
        cap=cv2.VideoCapture(cid,back)
        if cap.isOpened(): break
    else: raise RuntimeError(f'相机 {cid} 打不开')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,0)
    cap.set(cv2.CAP_PROP_EXPOSURE, 20)   # 曝光时间，尽量提高帧率
    cap.set(cv2.CAP_PROP_GAIN,6)
    print('实际帧率', cap.get(cv2.CAP_PROP_FPS))
    print('实际曝光(100µs)', cap.get(cv2.CAP_PROP_EXPOSURE))
    return cap




def grab_thread(cap,q):
    while cap.isOpened():
        if cap.grab(): q.put(time.time())
        else: time.sleep(0.001)

# ---------- 主循环 ----------
def sync_record(lcap,rcap):
    os.makedirs(OUT_DIR,exist_ok=True)
    lq,rq=queue.Queue(maxsize=5),queue.Queue(maxsize=5)
    threading.Thread(target=grab_thread,args=(lcap,lq),daemon=True).start()
    threading.Thread(target=grab_thread,args=(rcap,rq),daemon=True).start()

    frame_buffer=[]   # [(frame, t), ...]
    recording=False
    start_t=None

    while True:
        try:
            tL,tR=lq.get(timeout=0.2),rq.get(timeout=0.2)
        except queue.Empty: continue
        # 可继续保留同步判断，也可注释掉
        if abs(tL-tR)>0.1: continue
        ret1,imgL=lcap.retrieve()
        ret2,imgR=rcap.retrieve()
        if not (ret1 and ret2): continue

        frame=cv2.hconcat([imgL,imgR])
        cv2.imshow('vfr-stereo (ESC退出, 空格录制/暂停)', cv2.resize(frame,(1280,360)))

        key=cv2.waitKey(1)&0xFF
        if key==27: break
        if key==ord(' '):
            recording=not recording
            if recording and start_t is None:
                start_t=time.time()
                print('[>] 开始录制...')
            print('[pause]' if not recording else '[recording]')

        if recording:
            frame_buffer.append((frame.copy(), time.time()))

    # 计算平均帧率并写文件
    if frame_buffer:
        fps = len(frame_buffer) / (frame_buffer[-1][1] - frame_buffer[0][1])
        ts=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path=os.path.join(OUT_DIR,f'stereo_vfr_{ts}.mp4')
        fourcc=cv2.VideoWriter_fourcc(*'mp4v')
        h,w=frame_buffer[0][0].shape[:2]
        writer=cv2.VideoWriter(out_path,fourcc,fps,(w,h))
        for frm,_ in frame_buffer:
            writer.write(frm)
        writer.release()
        print(f'[v] 保存完成  {out_path}')
        print(f'    总帧数={len(frame_buffer)}  平均fps={fps:.2f}  时长={len(frame_buffer)/fps:.2f}s')
    cv2.destroyAllWindows()

# ---------- main ----------
def main():
    cams=list_cameras()
    print('可用相机:',cams)
    if len(cams)<2: raise RuntimeError('不足 2 台')
    left,right=open_camera(LEFT_ID),open_camera(RIGHT_ID)
    print(f'使用 LEFT={LEFT_ID}  RIGHT={RIGHT_ID}')
    sync_record(left,right)
    left.release(); right.release()

if __name__=='__main__':
    main()