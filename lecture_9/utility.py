import numpy as np
import open3d as o3d


def read_oxford_bin(path):
    data_np = np.fromfile(path, dtype=np.float32)
    return np.reshape(data_np, (int(data_np.shape[0] / 6), 6))


def display_oxford_pointcloud(pointcloud: np.ndarray):
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    o3d.visualization.draw_geometries([pointcloud_o3d])


def display_oxford_pointcloud_with_normal(pointcloud: np.ndarray):
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    pointcloud_o3d.normals = o3d.utility.Vector3dVector(pointcloud[:, 3:])
    o3d.visualization.draw_geometries([pointcloud_o3d], point_show_normal=True)


if __name__ == '__main__':
    file_path = '/home/meng/bag/registration_dataset/point_clouds/0.bin'
    pointcloud_np = read_oxford_bin(file_path)
    display_oxford_pointcloud(pointcloud_np)
