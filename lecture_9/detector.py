import open3d as o3d
import utility
import argparse


def detect_ISS(pointcloud):
    """
    detect iss keypoints
    :param pointcloud: o3d.geometry.Pointcloud()
    :return:
    """
    keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pointcloud)
    return keypoints


def get_arguments():
    parser = argparse.ArgumentParser(description='detect pointcloud keypoints')
    parser.add_argument('-f', '--file_path', required=True, help='file path')
    return parser.parse_args()


if __name__ == '__main__':
    arguments = get_arguments()

    pointcloud_numpy = utility.read_oxford_bin(arguments.file_path)
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_numpy[:, 0:3])
    keypoints = detect_ISS(pointcloud_o3d)
    o3d.visualization.draw_geometries([keypoints])
