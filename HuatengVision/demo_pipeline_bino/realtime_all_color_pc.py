# realtime_all_color_pc.py
import cv2, numpy as np, open3d as o3d, camera_configs as cfg, threading, queue, time, test_dual_cam_opt2 as cam
from datetime import datetime

# ---------- 相机参数 ----------
# b, f = 84.89, 963          #  baseline(mm), 焦距(pixel)  如标定有变自行改
# cx, cy = cfg.P1[0,2], cfg.P1[1,2]  # 主点用新标定矩阵
b, f, cx, cy = cfg.b, cfg.f, cfg.cx, cfg.cy

# ---------- SGBM ----------
num = 3
blockSize = 13
sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16*num,
        blockSize=blockSize,
        P1=8*3*blockSize**2,
        P2=32*3*blockSize**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32)

def dis_to_3d(dis, rgb):
    dis = dis.astype(np.float32) / 16.0          # 转真实视差
    mask = (dis > 1.) & (dis < 250)              # 按实际范围调
    z = b * f / dis[mask]
    c, r = np.meshgrid(np.arange(dis.shape[1]), np.arange(dis.shape[0]))
    x = z * (c[mask] - cx) / f
    y = z * (r[mask] - cy) / f
    pts = np.stack([x, y, z, rgb[mask, 2], rgb[mask, 1], rgb[mask, 0]], 1)
    return pts

def show_pcd(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:,:3])
    pcd.colors  = o3d.utility.Vector3dVector(pts[:,3:6]/255.)
    o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    # ---------- 主循环 ----------
    print('空格=冻结并生成点云，ESC 退出')
    running = True
    while running:
        imgL, imgR = cam.get_sync_frame()
        if imgL is None: continue
        rL = cv2.remap(imgL, cfg.left_map1, cfg.left_map2, cv2.INTER_LINEAR)
        rR = cv2.remap(imgR, cfg.right_map1, cfg.right_map2, cv2.INTER_LINEAR)
        grayL = cv2.cvtColor(rL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(rR, cv2.COLOR_BGR2GRAY)
        dis = sgbm.compute(grayL, grayR)
        dis_vis = cv2.normalize(dis, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        dis_color = cv2.applyColorMap(dis_vis, cv2.COLORMAP_JET)
        cv2.imshow('left', rL)
        cv2.imshow('right', rR)
        cv2.imshow('depth', dis_color)

        key = cv2.waitKey(1) & 0xff
        if key == 27:                      # ESC → 真正退出
            running = False
            break
        if key == ord(' '):
            # 获取当前时间戳并格式化为字符串
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # 将时间戳添加到文件名中
            file_name = f"debug_disp_{timestamp}.png"
            # 保存视差图供肉眼检查
            cv2.imwrite(file_name, dis_vis)
            pts = dis_to_3d(dis, rL)
            show_pcd(pts)

    # 释放资源 **一次就够**
    cv2.destroyAllWindows()
    # 如果 test_dual_cam_opt2 里 capL/R 是全局的，也在这里 release
    cam.capL.release()
    cam.capR.release()
    print('程序正常结束')