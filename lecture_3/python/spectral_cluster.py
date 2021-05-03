# 文件功能： 实现 Spectral Clustering 算法

import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt


class SpectralClustering(object):
    def __init__(self, n_clusters=2, **kwargs):
        self.__K = n_clusters
        self.__labels = None

    def fit(self, data):
        from sklearn.metrics import pairwise_distances
        from scipy.sparse import csgraph

        N, _ = data.shape

        A = pairwise_distances(data)
        gamma = np.var(A) / 4
        A = np.exp(-A ** 2 / (2 * gamma ** 2))
        L = csgraph.laplacian(A, normed=True)
        eigval, eigvec = np.linalg.eig(L)
        idx_k_smallest = np.where(eigval < np.partition(eigval, self.__K)[self.__K])
        features = np.hstack([eigvec[:, i] for i in idx_k_smallest])
        k_means = KMeans(init='k-means++', n_clusters=self.__K, tol=1e-6)
        k_means.fit(features)
        self.__labels = k_means.labels_

    def predict(self, data):
        return np.copy(self.__labels)


def generate_dataset(N=300, noise=0.07, random_state=42, visualize=False):
    from sklearn.datasets import make_moons

    X, y = make_moons(N, noise=noise, random_state=random_state)

    if visualize:
        fig, ax = plt.subplots(figsize=(16, 9))
        ax.set_title('Test Dataset for Spectral Clustering', fontsize=18, fontweight='demi')
        ax.scatter(X[:, 0], X[:, 1], c=y, s=50, cmap='viridis')
        plt.show()

    return X


if __name__ == '__main__':
    K = 2
    X = generate_dataset(visualize=False)

    sc = SpectralClustering(n_clusters=K)
    sc.fit(X)

    category = sc.predict(X)

    color = ['red', 'blue', 'green', 'cyan', 'magenta']
    labels = [f'Cluster{k:02d}' for k in range(K)]

    for k in range(K):
        plt.scatter(X[category == k][:, 0], X[category == k][:, 1], c=color[k], label=labels[k])

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Spectral Clustering Test')
    plt.show()
