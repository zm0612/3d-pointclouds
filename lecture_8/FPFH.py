import numpy as np
import open3d as o3d
from matplotlib import pyplot as plt
import utility


def compute_SPFH(pointcloud, tree, point_index, radius, B):
    """
    Using Simplified Point Feature Histogram(SPFH)
    :param pointcloud: numpy.darray
        input point cloud
    :param tree: Open3d.geometry.KDTreeFlann
        point cloud search tree
    :param point_index: int
        keypoint index
    :param radius: float
        nearest neighborhood radius
    :param B: float
        number of bins for each dimension
    :return:
    """
    alpha, phi, theta = [], [], []
    point = pointcloud[point_index][0:3]
    nei_labels = tree.search_radius_vector_3d(point[0:3], radius)[1]
    nei_labels = np.asarray(list(set(nei_labels) - {point_index}))
    local_points = pointcloud[nei_labels]

    u = pointcloud[point_index][3:]
    p1 = pointcloud[point_index][0:3]

    for neighbor in local_points:
        p2 = neighbor[0:3]
        p2_p1_normal = (p2 - p1) / np.linalg.norm(p2 - p1, ord=2)
        v = np.cross(u, p2_p1_normal)
        w = np.cross(u, v)
        alpha.append(np.dot(v, neighbor[3:]))
        phi.append(np.dot(u, p2_p1_normal))
        theta.append(np.arctan2(np.dot(w, neighbor[3:]), np.dot(u, neighbor[3:])))

    alpha_histogram = np.histogram(alpha, B, range=(-1.0, 1.0))[0]
    alpha_histogram = alpha_histogram / alpha_histogram.sum()
    phi_histogram = np.histogram(phi, B, range=(-1.0, 1.0))[0]
    phi_histogram = phi_histogram / phi_histogram.sum()
    theta_histogram = np.histogram(theta, B, range=(-np.pi, np.pi))[0]
    theta_histogram = theta_histogram / theta_histogram.sum()
    signature = np.hstack(
        (
            alpha_histogram,
            phi_histogram,
            theta_histogram
        )
    )

    return signature


def compute_FPFH(pointcloud, tree, point_index, radius, B):
    """
    Compute Fast Point Feature Histogram(FPFH)
    :param pointcloud: numpy.darray
        input point cloud
    :param tree: Open3d.geometry.KDTreeFlann
        point cloud search tree
    :param point_index: int
        keypoint index
    :param radius: float
        nearest neighborhood radius
    :param B: float
        number of bins for each dimension
    :return:
    """
    point = pointcloud[point_index][0:3]
    nei_labels = tree.search_radius_vector_3d(point[0:3].reshape(3, 1), radius)[1]
    nei_labels = nei_labels[1:]

    w = 1.0 / np.linalg.norm(pointcloud[nei_labels, 0:3] - point, ord=2, axis=1)
    X = np.asarray(
        [compute_SPFH(pointcloud, tree, i, radius, B) for i in nei_labels]
    )
    spfh_nei = 1.0 / (len(nei_labels)) * np.dot(w, X)
    spfh_query = compute_SPFH(pointcloud, tree, point_index, radius, B)

    spfh = spfh_nei + spfh_query
    spfh = spfh / np.linalg.norm(spfh)

    return spfh


if __name__ == '__main__':
    file_path = 'chair_data/chair_0001.txt'
    point_cloud_list = utility.read_pointcloud_with_normal(file_path)

    point_cloud_numpy = np.asarray(point_cloud_list)
    point_cloud_o3d = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(point_cloud_numpy[:, 0:3]))

    tree = o3d.geometry.KDTreeFlann(point_cloud_o3d)

    query_point_index = 3705
    query_point_index_1 = 1000

    spfh = compute_FPFH(point_cloud_numpy, tree, query_point_index, 0.05, 8)
    spfh_1 = compute_FPFH(point_cloud_numpy, tree, query_point_index_1, 0.05, 8)
    plt.plot(spfh, 'r')
    plt.plot(spfh_1, 'b')
    plt.show()
    query_point = o3d.geometry.PointCloud()
    query_point.points = o3d.utility.Vector3dVector(
        point_cloud_numpy[[query_point_index, query_point_index_1], 0:3].reshape(2, -1))
    point_cloud_o3d.paint_uniform_color([0, 1, 1])
    query_point.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([point_cloud_o3d, query_point])
