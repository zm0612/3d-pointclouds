import open3d as o3d
import numpy as np
import argparse
import detector
import descriptor
import utility


def get_matches(feature_target, feature_source):
    """
    get the matching relationship through the nearest neighbor search of the descriptor.
    Only when they are the nearest neighbors will they be recorded
    :param feature_target: open3d.pipeline.registration.Feature
    :param feature_source: open3d.pipeline.registration.Feature
    :return: numpy.ndarray N x 2 [source_index, target_index]
    """
    search_tree_target = o3d.geometry.KDTreeFlann(feature_target)
    search_tree_source = o3d.geometry.KDTreeFlann(feature_source)
    _, N = feature_source.data.shape
    matches = []

    for i in range(N):
        query_source = feature_source.data[:, i]
        _, nn_target_index, _ = search_tree_target.search_knn_vector_xd(query_source, 1)

        query_target = feature_target.data[:, nn_target_index[0]]
        _, nn_source_index, _ = search_tree_source.search_knn_vector_xd(query_target, 1)

        if nn_source_index[0] == i:
            matches.append([i, nn_target_index[0]])

    matches = np.asarray(matches)
    return matches


def ransac_registration(keypoints_target, keypoints_source, matches, max_iteration, max_corr_dist):
    """
    conduct icp registration with RANSAC
    :param keypoints_target: [open3d.geometry.PointCloud]
    :param keypoints_source: [open3d.geometry.PointCloud]
    :param matches: descriptor index of similarity between source and target [np.ndarray N x 2]
    :param max_iteration: number of RANSAC iterations [int]
    :param max_corr_dist: maximum matching point distance [float]
    :return: best T and best fitness score
    """
    keypoints_target_numpy = np.asarray(keypoints_target.points)
    keypoints_source_numpy = np.asarray(keypoints_source.points)

    N, _ = matches.shape
    idx_matches = np.arange(N)

    best_T = np.zeros((4, 4))
    best_score = -np.inf

    for i in range(max_iteration):
        pairs = np.random.choice(idx_matches, 3, replace=False)
        P = keypoints_target_numpy[matches[pairs][:, 1]]
        Q = keypoints_source_numpy[matches[pairs][:, 0]]
        T = compute_ICP(P.T, Q.T)
        registration_result = o3d.pipelines.registration.evaluate_registration(
            keypoints_target, keypoints_source, max_corr_dist, T
        )
        if registration_result.fitness > best_score:
            best_T = T
            best_score = registration_result.fitness
    return best_T, best_score


def compute_ICP(A, B):
    """
    compute ICP
    :param A: target pointcloud [np.ndarray 3 x N]
    :param B: source pointcloud [np.ndarray 3 x N]
    :return: T (source to target)
    """
    _, N = A.shape
    L = np.identity(N) - 1.0 / N * np.ones((N, 1)) * np.ones((1, N))
    A_p = np.dot(A, L)
    B_p = np.dot(B, L)
    U, s, V = np.linalg.svd(np.dot(B_p, A_p.T), full_matrices=True)
    R = np.dot(U, V)
    t = 1.0 / N * np.dot((B - np.dot(R, A)), np.ones((N, 1)))

    T = np.zeros((4, 4))
    T[0:3, 0:3] = R
    T[0:3, 3] = t.T
    T[3, 3] = 1.0

    return T


def get_arguments():
    parser = argparse.ArgumentParser("point cloud registration")
    parser.add_argument("-f", "--file_path", nargs=2, required=True, help="point cloud file path")
    parser.add_argument("-i", "--iterations", type=int, help="ransac icp iterations")
    return parser.parse_args()


if __name__ == '__main__':
    arguments = get_arguments()

    # read target pointcloud
    pointcloud_numpy_target = utility.read_oxford_bin(arguments.file_path[0])
    pointcloud_o3d_target = o3d.geometry.PointCloud()
    pointcloud_o3d_target.points = o3d.utility.Vector3dVector(pointcloud_numpy_target[:, 0:3])
    pointcloud_o3d_target.normals = o3d.utility.Vector3dVector(pointcloud_numpy_target[:, 3:6])
    # pointcloud_o3d_target, _ = pointcloud_o3d_target.remove_radius_outlier(nb_points=4, radius=0.5)
    # compute target keypoints and descriptors
    keypoints_target = detector.detect_ISS(pointcloud_o3d_target)
    descriptor_target = descriptor.compute_fpfh(keypoints_target)

    # read source pointcloud
    pointcloud_numpy_source = utility.read_oxford_bin(arguments.file_path[1])
    pointcloud_o3d_source = o3d.geometry.PointCloud()
    # To facilitate observation, move the point cloud upward by 20 meters
    pointcloud_o3d_source.points = o3d.utility.Vector3dVector(pointcloud_numpy_source[:, 0:3] + np.array([0, 0, 20]))
    pointcloud_o3d_source.normals = o3d.utility.Vector3dVector(pointcloud_numpy_source[:, 3:6])
    # pointcloud_o3d_source, _ = pointcloud_o3d_source.remove_radius_outlier(nb_points=4, radius=0.5)
    # compute source keypoints and descriptors
    keypoints_source = detector.detect_ISS(pointcloud_o3d_source)
    descriptor_source = descriptor.compute_fpfh(keypoints_source)

    # get the matching relationship through the nearest neighbor search of the descriptor
    matches = get_matches(descriptor_target, descriptor_source)
    T, score = ransac_registration(keypoints_target, keypoints_source, matches, arguments.iterations, 2.0)
    print("\nThe number of descriptor matches", len(matches))
    print(" score: ", score)
    print("\n rough T (Only RANSAC): \n", T)

    # display original pointcloud
    o3d.visualization.draw_geometries([pointcloud_o3d_target, pointcloud_o3d_source], 'original')

    # display registration result 
    # To facilitate observation, move the point cloud upward by 20 meters
    keypoints_source.paint_uniform_color([0, 1, 0])
    keypoints_target.paint_uniform_color([1, 0, 0])
    utility.display_registration_result(
        keypoints_source=keypoints_source,
        keypoints_target=keypoints_target,
        matches=matches,
        pointcloud_source=pointcloud_o3d_source,
        pointcloud_target=pointcloud_o3d_target
    )

    # Use the RANSAC method to obtain the matching result as the initial value,
    # and perform fine icp matching
    icp = o3d.pipelines.registration.registration_icp(
        pointcloud_o3d_target, pointcloud_o3d_source, 1.0, T
    )
    print("\n fine ICP result:\n", icp.transformation)

    # transform source pointcloud by T.inv
    transformed_pointcloud_source = pointcloud_o3d_source.transform(np.linalg.inv(icp.transformation))

    # set color
    transformed_pointcloud_source.paint_uniform_color([0, 1, 0])
    pointcloud_o3d_target.paint_uniform_color([1, 0, 0])

    # display fine icp matched result
    o3d.visualization.draw_geometries([transformed_pointcloud_source, pointcloud_o3d_target], 'matched')
