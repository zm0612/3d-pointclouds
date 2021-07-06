import open3d as o3d
import numpy as np
import pandas
import argparse
import utility
import detector


def compute_fpfh(
        pointcloud_with_normal,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=5, max_nn=100)
):
    feature_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pointcloud_with_normal, search_param=search_param)
    return feature_fpfh


def get_arguments():
    parser = argparse.ArgumentParser(description='compute pointcloud descriptor')
    parser.add_argument('-f', '--file_path', required=True, help='file path')
    parser.add_argument('-t', '--descriptor_type', required=True, choices=['fpfh'], help='choice descriptor type')
    return parser.parse_args()


if __name__ == '__main__':
    arguments = get_arguments()

    pointcloud_numpy = utility.read_oxford_bin(arguments.file_path)
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_numpy[:, 0:3])
    pointcloud_o3d.normals = o3d.utility.Vector3dVector(pointcloud_numpy[:, 3:6])

    keypoints = detector.detect_ISS(pointcloud_o3d)
    descriptor = compute_fpfh(keypoints)
    print(descriptor)
    print(descriptor.data.shape)
