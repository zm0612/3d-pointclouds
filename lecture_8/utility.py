import open3d as o3d
import numpy as np


def read_pointcloud_with_normal(path):
    point_cloud = []
    with open(path) as file:
        for line in file.readlines():
            point = [float(value) for value in line.split(',')]
            point_cloud.append(point)

    return np.array(point_cloud)


def display_pointcloud(point_cloud):
    """
    通过open3d进行点云显示
    :param point_cloud: 列表格式的点云数据
    :return: void
    """
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(point_cloud)
    o3d.visualization.draw_geometries([pointcloud_o3d])


def display_pointcloud_with_normal(point_cloud):
    """
    通过open3d进行点云显示，包含法向量的显示
    :param point_cloud: 列表格式的点云数据
    :return: void
    """
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(point_cloud[:, 0:3])
    pointcloud_o3d.normals = o3d.utility.Vector3dVector(point_cloud[:, 3:6])
    o3d.visualization.draw_geometries([pointcloud_o3d], point_show_normal=True)


