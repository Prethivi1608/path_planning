import numpy as np
import open3d as o3d

#loading PCD File
pcd = o3d.io.read_point_cloud("/home/prethivi/ros2_ws/pathplanning/room_scan2.pcd")
print(pcd)

#downsampling
pcd_down = pcd.voxel_down_sample(voxel_size = 0.5)

print(pcd_down)

#filtering
cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd_filtered = pcd_down.select_by_index(ind)

#write a new pcd file
o3d.io.write_point_cloud("processed.pcd", pcd_filtered)