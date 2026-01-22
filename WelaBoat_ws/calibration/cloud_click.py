import open3d as o3d
import json

pcd = o3d.io.read_point_cloud("cloud.pcd")
print("Shift + click to pick points, Q to finish")

vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(pcd)
vis.run()
vis.destroy_window()

idx = vis.get_picked_points()
pts = [pcd.points[i] for i in idx]

with open("cloud_points.json", "w") as f:
    json.dump([list(p) for p in pts], f)
