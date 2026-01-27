import open3d as o3d
import numpy as np
import pandas as pd
import sys

def csv_to_pcd(csv_path, pcd_path):
    # 读取CSV：假设每行8列，无表头
    df = pd.read_csv(
        csv_path,
        header=None,          # 无列名
        usecols=[0, 1, 2],    # 只读 x, y, z
        names=['x', 'y', 'z'],
        dtype=np.float32,
        skip_blank_lines=True
    )
    
    # 过滤无效点（可选）
    points = df[['x', 'y', 'z']].values
    valid = np.all(np.isfinite(points), axis=1)
    points = points[valid]
    
    print(f"Loaded {points.shape[0]} points from {csv_path}")

    # 创建并保存点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"Saved to {pcd_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python csv_to_pcd.py input.csv output.pcd")
        sys.exit(1)
    csv_to_pcd(sys.argv[1], sys.argv[2])