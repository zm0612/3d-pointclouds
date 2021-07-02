import numpy as np
import open3d as o3d
import argparse


def read_oxford_bin(path):
    """
    read point cloud from oxford bin file
    :param path: file path
    :return: N x 6 (first three: x, y, z  last three:nx, ny, nz)
    """
    data_np = np.fromfile(path, dtype=np.float32)
    return np.reshape(data_np, (int(data_np.shape[0] / 6), 6))


def display_oxford_pointcloud(pointcloud: np.ndarray):
    """
    display pointcloud
    :param pointcloud: numpy.ndarray
    :return: None
    """
    pointcloud_o3d = o3d.geometry.PointCloud()
    if pointcloud.shape[1] > 3:
        pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    else:
        pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud)
    o3d.visualization.draw_geometries([pointcloud_o3d])


def display_oxford_pointcloud_with_normal(pointcloud: np.ndarray):
    """
    display pointcloud and normals
    :param pointcloud: N x 6 numpy.ndarray
    :return:
    """
    if pointcloud.shape[1] != 6:
        print('Please input correct pointcloud')
        exit(-1)
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
    pointcloud_o3d.normals = o3d.utility.Vector3dVector(pointcloud[:, 3:])
    o3d.visualization.draw_geometries([pointcloud_o3d], point_show_normal=True)


def get_arguments():
    parser = argparse.ArgumentParser(description="utility of reading and displaying point cloud")
    parser.add_argument('-f', '--file_path', help='point cloud file path', required=True, type=str)
    parser.add_argument('-d', '--display', action='store_true', help='need to display the point cloud')
    parser.add_argument('-n', '--normal', action='store_true',
                        help='display the normals, need to set -d and -n at the same time')
    return parser.parse_args()


if __name__ == '__main__':
    args = get_arguments()

    point_cloud_np = read_oxford_bin(args.file_path)

    if args.display and args.normal:
        display_oxford_pointcloud_with_normal(point_cloud_np)
    elif args.display:
        display_oxford_pointcloud(point_cloud_np)
