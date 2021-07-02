import open3d as o3d
import numpy as np


def get_matches(feature_target, feature_source):
    """
    query matched pair
    :param feature_target: open3d.pipeline.registration.Feature
    :param feature_source: open3d.pipeline.registration.Feature
    :return: numpy.ndarray N x 2
    """
    search_tree = o3d.geometry.KDTreeFlann(feature_target)
    _, N = feature_source.data.shape
    matches = []

    for i in range(N):
        query = feature_source[:, i]
        _, nn_target_index, _ = search_tree.search_knn_vector_3d(query, 1)
        matches.append([i, nn_target_index[0]])

    matches = np.asarray(matches)
    return matches


def ransac_registration(pointcloud_target, pointcloud_source, matches, max_iteration):
    pointcloud_source_numpy = np.asarray(pointcloud_source.points)
