import open3d as o3d
import numpy as np


class ISS:
    def __init__(self, knn=20, gamma_21=0.975, gamma_32=0.975):
        self.knn = knn
        self.gamma_21 = gamma_21
        self.gamma_32 = gamma_32

    def compute_keypoints(self, pointcloud):
        point = np.asarray(pointcloud.points)
        point_size = point.shape[0]
        picked_point = [0 for x in range(point_size)]
        iss_keypoints = []
        mean_knn_source = np.zeros((point_size, self.knn, 3))
        mean_keypoint_source = np.zeros((point_size, 3))
        for i in range(point_size):
            if picked_point[i] == 1:
                continue

            tree = o3d.geometry.KDTreeFlann(pointcloud)
            [_, idx, dis] = tree.search_knn_vector_3d(point[i], self.knn + 1)
            mean_keypoint_source[i] = point[i]
            mean_knn_source[i] = point[idx[1:]]
            d_vector = mean_knn_source[i] - mean_keypoint_source[i]
            C_tem = np.sum(np.reshape((np.sum(d_vector[-1] ** 2) ** (1 / 2) - np.sum(d_vector ** 2, axis=1) ** (1 / 2)),
                                      (self.knn, 1, 1)) * np.matmul(np.expand_dims(d_vector, axis=-1),
                                                                    np.transpose(np.expand_dims(d_vector, axis=-1),
                                                                                 (0, 2, 1))), axis=0) / np.sum(
                (np.sum(d_vector[-1] ** 2) ** (1 / 2) - np.sum(d_vector[1:] ** 2, axis=1) ** (1 / 2)))
            eigvalue, eigvector = np.linalg.eig(C_tem)
            eigvalue = np.sort(eigvalue)

            if eigvalue[1] / (eigvalue[2] + 1e-5) <= self.gamma_21 and eigvalue[0] / (
                    eigvalue[1] + 1e-5) <= self.gamma_32:
                iss_keypoints.append(point[i])

            keypoints = o3d.geometry.PointCloud()
            keypoints.points = o3d.utility.Vector3dVector(iss_keypoints)

            for id in idx:
                picked_point[id] = 1

        return keypoints
