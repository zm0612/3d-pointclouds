import iss
import utility
import open3d as o3d
import numpy as np
import time

path = "/home/meng/bag/ModelNet40/airplane/test/airplane_0627.off"
path_pcd = "/home/meng/bag/key_frame_0.pcd"
pointcloud = o3d.geometry.PointCloud()
# pointcloud = o3d.io.read_point_cloud(path_pcd)
pointcloud.points = o3d.utility.Vector3dVector(utility.read_pointcloud(path))
keypoints = o3d.geometry.PointCloud()
pointcloud.paint_uniform_color([0, 0, 0])

# my iss
iss_detector = iss.ISS()
my_iss_time_start = time.time()
keypoints = iss_detector.compute_keypoints(pointcloud)
my_iss_time_end = time.time()
print('my time cost', my_iss_time_end - my_iss_time_start, 's')
keypoints.paint_uniform_color([1.0, 0, 0])

# open3d iss
o3d_iss_time_start = time.time()
keypoints_o3d = o3d.geometry.keypoint.compute_iss_keypoints(pointcloud)
o3d_iss_time_end = time.time()
print('o3d time cost', o3d_iss_time_end - o3d_iss_time_start, 's')
keypoints_o3d.paint_uniform_color([1.0, 0, 0])

o3d.visualization.draw_geometries([pointcloud, keypoints_o3d], window_name="open_3d")
