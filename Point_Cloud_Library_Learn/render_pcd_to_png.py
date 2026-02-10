# render_pcd_to_png.py
import open3d as o3d
import numpy as np
import sys
import os

def colorize_by_height(pcd):
    """根据点的 Z 坐标上色（模拟 pcl_viewer 的默认效果）"""
    points = np.asarray(pcd.points)
    if len(points) == 0:
        return pcd

    z_vals = points[:, 2]
    z_min, z_max = z_vals.min(), z_vals.max()
    if z_max == z_min:
        colors = np.tile([0.5, 0.5, 0.5], (len(points), 1))  # 灰色
    else:
        normalized_z = (z_vals - z_min) / (z_max - z_min)
        # 使用 colormap: blue (low) -> green -> red (high)
        colors = np.zeros((len(points), 3))
        colors[:, 0] = np.clip(normalized_z * 2 - 1, 0, 1)      # Red
        colors[:, 1] = np.clip(1 - np.abs(normalized_z * 2 - 1), 0, 1)  # Green
        colors[:, 2] = np.clip(1 - normalized_z * 2, 0, 1)      # Blue
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def main():
    if len(sys.argv) != 3:
        print("用法: python render_pcd_to_png.py <input.pcd> <output.png>")
        sys.exit(1)

    pcd_path = sys.argv[1]
    png_path = sys.argv[2]

    # 加载点云
    pcd = o3d.io.read_point_cloud(pcd_path)
    if len(pcd.points) == 0:
        print(f"警告: {pcd_path} 是空点云")
        blank = np.ones((720, 1280, 3), dtype=np.uint8) * 255
        o3d.io.write_image(png_path, o3d.geometry.Image(blank))
        return

    # 如果无颜色，按高度上色（关键！）
    if not pcd.has_colors():
        pcd = colorize_by_height(pcd)

    # 计算点云中心和尺度
    center = pcd.get_center()
    bbox = pcd.get_axis_aligned_bounding_box()
    max_dim = max(bbox.get_extent())

    # 创建可视化器（离屏）
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1280, height=720, visible=False)
    vis.add_geometry(pcd)

    # 设置视角：从斜上方看，距离与点云大小成比例
    ctr = vis.get_view_control()
    ctr.set_lookat(center)                          # 注视点云中心
    camera_pos = center + np.array([max_dim, -max_dim, max_dim * 0.8])
    ctr.set_front(camera_pos - center)              # 相机方向
    ctr.set_up([0, 0, 1])                           # Z轴向上
    ctr.set_zoom(0.5)                               # 适当缩放

    # 关键：设置点大小（否则点太小看不见）
    render_opt = vis.get_render_option()
    render_opt.point_size = 2.0                     # 可调：1~5
    render_opt.background_color = np.asarray([0, 0, 0])  # 黑色背景，更清晰

    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(png_path, do_render=True)
    vis.destroy_window()

if __name__ == "__main__":
    main()