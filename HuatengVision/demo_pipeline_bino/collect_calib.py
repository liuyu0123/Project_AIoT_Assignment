#!/usr/bin/env python3
import cv2, os, time, test_dual_cam_opt2 as cam  # 复用刚才的 DSHOW 线程模板

SAVE_L = 'left'; SAVE_R = 'right'
os.makedirs(SAVE_L, exist_ok=True); os.makedirs(SAVE_R, exist_ok=True)

count = 0
print('空格拍照，ESC 结束')
while True:
    imgL, imgR = cam.get_sync_frame()
    if imgL is None: continue
    cv2.imshow('left', cv2.resize(imgL,(640,360)))
    cv2.imshow('right',cv2.resize(imgR,(640,360)))
    key = cv2.waitKey(1)
    if key == 27: break
    if key == ord(' '):
        cv2.imwrite(f'{SAVE_L}/{count:02d}.png', imgL)
        cv2.imwrite(f'{SAVE_R}/{count:02d}.png', imgR)
        print(f'saved {count:02d}'); count += 1
cv2.destroyAllWindows()