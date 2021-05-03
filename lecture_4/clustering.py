import argparse
import os
import glob
import random
import time
import struct
import numpy as np
import open3d as o3d
import pcl
from sklearn.cluster import DBSCAN


def voxel_grid_filter(data):
    pointcloud = pcl.PointCloud(data)
    voxel_filter = pointcloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(0.2, 0.2, 0.2)
    pointcloud_filtered = voxel_filter.filter()

    return pointcloud_filtered


def clustering(data):
    cluster_index = DBSCAN(
        eps=0.25, min_samples=5, n_jobs=-1
    ).fit_predict(data)

    return cluster_index


class GroundSegmenter():
    def __init__(self, cloud, max_distance=0.30):
        self.__max_distance = max_distance
        self.__segmenter = cloud.make_segmenter()
        self.__segmenter.set_model_type(pcl.SACMODEL_PLANE)
        self.__segmenter.set_method_type(pcl.SAC_RANSAC)
        self.__segmenter.set_distance_threshold(self.__max_distance)
        self.__segmenter.set_optimize_coefficients(True)

    def get_max_distance(self):
        return self.__max_distance

    def segment(self):
        return self.__segmenter.segment()


def ground_segmentation(data):
    pcd_original = o3d.geometry.PointCloud()
    pcd_original.points = o3d.utility.Vector3dVector(data)
    pcd_original.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=5.0, max_nn=9
        )
    )
    normals = np.asarray(pcd_original.normals)
    angular_distance_to_z = np.abs(normals[:, 2])
    idx_downsampled = angular_distance_to_z > np.cos(np.pi / 8)
    downsampled = data[idx_downsampled]

    cloud = pcl.PointCloud()
    cloud.from_array(downsampled)
    ground_segmenter = GroundSegmenter(cloud=cloud)
    inliers, model = ground_segmenter.segment()

    distance_to_ground = np.abs(
        np.dot(data, np.asarray(model[:3])) + model[3]
    )

    idx_ground = distance_to_ground <= ground_segmenter.get_max_distance()
    idx_segmented = np.logical_not(idx_ground)
    segmented_cloud = data[idx_segmented]
    segmented_ground = data[idx_ground]

    return segmented_cloud, segmented_ground


def plot_cluster(segmented_ground, segmented_cloud, cluster_index):
    def colormap(c, num_clusters):
        if c == -1:
            color = [1] * 3
        else:
            color = [0] * 3
            color[c % 3] = c / num_clusters
        return color

    pcd_ground = o3d.geometry.PointCloud()
    pcd_ground.points = o3d.utility.Vector3dVector(segmented_ground)
    pcd_ground.colors = o3d.utility.Vector3dVector(
        [
            [0.372] * 3 for i in range(segmented_ground.shape[0])
        ]
    )

    pcd_objects = o3d.geometry.PointCloud()
    pcd_objects.points = o3d.utility.Vector3dVector(segmented_cloud)
    num_clusters = max(cluster_index) + 1
    pcd_objects.colors = o3d.utility.Vector3dVector(
        [
            colormap(c, num_clusters) for c in cluster_index
        ]
    )

    o3d.visualization.draw_geometries([pcd_ground, pcd_objects])


def read_velodyne_bin(path):
    point_cloud_list = []
    with open(path, 'rb') as f:
        content = f.read()
        point_cloud_iter = struct.iter_unpack('ffff', content)
        for idx, point in enumerate(point_cloud_iter):
            point_cloud_list.append([point[0], point[1], point[2]])

    return np.asarray(point_cloud_list, dtype=np.float32)


def main():
    # data_path = '/home/meng/bag/velodyne'
    data_path = '/home/meng/code_my/3d-pointclouds/lecture_2/data'
    pattern = os.path.join(data_path, '*.bin')
    examples = random.sample(glob.glob(pattern), 1)

    for example in examples:
        lidar_measurements = read_velodyne_bin(example)
        filter_pointcloud = voxel_grid_filter(lidar_measurements)
        lidar_measurements = np.asarray(filter_pointcloud)

        time_segment_start = time.time()
        segmented_cloud, segmented_ground = ground_segmentation(lidar_measurements)
        time_segment_end = time.time()
        print("segmentation use time: ", time_segment_end - time_segment_start)

        time_cluster_start = time.time()
        cluster_index = clustering(segmented_cloud)
        time_cluster_end = time.time()
        print("clustering use time: ", time_cluster_end - time_cluster_start)

        plot_cluster(segmented_ground, segmented_cloud, cluster_index)


if __name__ == '__main__':
    main()
