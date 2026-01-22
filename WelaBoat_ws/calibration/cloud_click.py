import open3d as o3d
import json

pcd = o3d.io.read_point_cloud("/home/riba/GitProject/LIUYU/WelaBoat_ws/Data/data_calib3/lidar/0040.pcd")
print("Shift + click to pick points, Q to finish")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

idx = vis.get_picked_points()
pts = [pcd.points[i] for i in idx]

with open("calib/cloud_points.json", "w") as f:
    json.dump([list(p) for p in pts], f)
