# 文件功能： 实现 K-Means 算法

import numpy as np
import random


class K_Means(object):
    # k是分组数；tolerance‘中心点误差’；max_iter是迭代次数
    def __init__(self, n_clusters=2, tolerance=0.0001, max_iter=300):
        self.k_ = n_clusters
        self.tolerance_ = tolerance
        self.max_iter_ = max_iter
        self.centers_ = None

    def fit(self, data):
        # 作业1
        centers = data[random.sample(range(data.shape[0]), self.k_)]
        last_centers = np.copy(centers)
        labels = [[] for i in range(self.k_)]

        for i in range(self.max_iter_):
            for idx, point in enumerate(data):
                diff_dist = np.linalg.norm(last_centers - point, axis=1)
                label_index = (np.argmin(diff_dist))
                labels[label_index].append(idx)

            for i_k in range(self.k_):
                points = data[labels[i_k], :]
                centers[i_k] = points.mean(axis=0)

            if np.sum(np.abs(centers - last_centers)) < self.tolerance_ * self.k_:
                print("kmeans has converged, at ", i)
                break
            last_centers = np.copy(centers)
        self.centers_ = centers

    def predict(self, p_datas):
        result = []

        for point in p_datas:
            diff = np.linalg.norm(self.centers_ - point, axis=1)
            result.append(np.argmin(diff))

        return result


if __name__ == '__main__':
    x = np.array([[1, 2], [1.5, 1.8], [5, 8], [8, 8], [1, 0.6], [9, 11]])
    k_means = K_Means(n_clusters=2)
    k_means.fit(x)

    cat = k_means.predict(x)
    print(cat)
