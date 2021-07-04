import open3d as o3d
import numpy as np
import argparse
import detector
import descriptor
import utility


def get_matches(feature_target, feature_source):
    """
    query matched pair
    :param feature_target: open3d.pipeline.registration.Feature
    :param feature_source: open3d.pipeline.registration.Feature
    :return: numpy.ndarray N x 2 [source_index, target_index]
    """
    search_tree = o3d.geometry.KDTreeFlann(feature_target)
    _, N = feature_source.data.shape
    matches = []

    for i in range(N):
        query = feature_source.data[:, i]
        _, nn_target_index, _ = search_tree.search_knn_vector_xd(query, 1)
        matches.append([i, nn_target_index[0]])

    matches = np.asarray(matches)
    return matches


def ransac_registration(keypoints_target, keypoints_source, matches, max_iteration, max_corr_dist):
    keypoints_target_numpy = np.asarray(keypoints_target.points)
    keypoints_source_numpy = np.asarray(keypoints_source.points)

    N, _ = matches.shape
    idx_matches = np.arange(N)

    best_T = np.zeros((4, 4))
    best_score = -np.inf

    for i in range(max_iteration):
        pairs = np.random.choice(idx_matches, 3, replace=False)
        Q = keypoints_target_numpy[matches[pairs][:, 1]]
        P = keypoints_source_numpy[matches[pairs][:, 0]]
        T = compute_ICP(P, Q)
        registration_result = o3d.pipelines.registration.evaluate_registration(
            keypoints_target, keypoints_source, max_corr_dist, T
        )
        if registration_result.fitness > best_score:
            best_T = T
            best_score = registration_result.fitness
            # print(registration_result.)
            # print(best_T)
            # print(best_score)
    return best_T, best_score


def compute_ICP(P, Q):
    center_P = P.mean(axis=0)
    center_Q = Q.mean(axis=0)
    P_normalized = P - center_P
    Q_normalized = Q - center_Q
    U, s, V = np.linalg.svd(np.dot(Q_normalized, P_normalized.T), full_matrices=True, compute_uv=True)
    R = np.dot(U, V.T)
    t = center_Q - np.dot(R, center_P)
    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    T[3, 3] = 1.0

    return T


def get_arguments():
    parser = argparse.ArgumentParser("point cloud registration")
    parser.add_argument("-f", "--file_path", nargs=2, required=True, help="point cloud file path")
    parser.add_argument("-i", "--iterations", type=int, help="ransac icp iterations")
    return parser.parse_args()


if __name__ == '__main__':
    arguments = get_arguments()

    pointcloud_numpy_target = utility.read_oxford_bin(arguments.file_path[0])
    pointcloud_o3d_target = o3d.geometry.PointCloud()
    pointcloud_o3d_target.points = o3d.utility.Vector3dVector(pointcloud_numpy_target[:, 0:3])
    pointcloud_o3d_target.normals = o3d.utility.Vector3dVector(pointcloud_numpy_target[:, 3:6])
    pointcloud_o3d_target, _ = pointcloud_o3d_target.remove_radius_outlier(nb_points=4, radius=0.5)

    keypoints_target = detector.detect_ISS(pointcloud_o3d_target)
    descriptor_target = descriptor.compute_fpfh(keypoints_target)

    pointcloud_numpy_source = utility.read_oxford_bin(arguments.file_path[1])
    pointcloud_o3d_source = o3d.geometry.PointCloud()
    pointcloud_o3d_source.points = o3d.utility.Vector3dVector(pointcloud_numpy_source[:, 0:3])
    pointcloud_o3d_source.normals = o3d.utility.Vector3dVector(pointcloud_numpy_source[:, 3:6])
    pointcloud_o3d_source, _ = pointcloud_o3d_source.remove_radius_outlier(nb_points=4, radius=0.5)

    keypoints_source = detector.detect_ISS(pointcloud_o3d_source)
    descriptor_source = descriptor.compute_fpfh(keypoints_source)

    matches = get_matches(descriptor_target, descriptor_source)
    T, score = ransac_registration(keypoints_target, keypoints_source, matches, arguments.iterations, 2.5)
    print("T: \n", T)
    print("\n score: ", score)

    o3d.visualization.draw_geometries([pointcloud_o3d_target, pointcloud_o3d_source])

    icp = o3d.pipelines.registration.registration_icp(
        pointcloud_o3d_source, pointcloud_o3d_target, 10.0, T
    )

    transformed_pointcloud_source = pointcloud_o3d_source.transform(icp.transformation)
    print(icp.transformation)
    transformed_pointcloud_source.paint_uniform_color([0, 1, 0])
    pointcloud_o3d_target.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([transformed_pointcloud_source, pointcloud_o3d_target])
