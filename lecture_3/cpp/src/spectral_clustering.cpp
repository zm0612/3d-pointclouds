//
// Created by Zhang Zhimeng on 2021/3/31.
//
#include "spectral_clustering.h"
#include "kmeans.h"
#include <iostream>

SpectralClustering::SpectralClustering(int K)
        : K_(K) {}

void SpectralClustering::Fit(const std::vector<Eigen::VectorXd> &source_points) {
    W_.resize(source_points.size(), source_points.size());
    for (unsigned int i = 0; i < source_points.size(); ++i) {
        for (unsigned int j = i; j < source_points.size(); ++j) {
            if (i == j) {
                W_(i, j) = 0;
            } else {
                W_(i, j) = 1.0 / (source_points[i] - source_points[j]).norm();
            }
        }
    }
    W_ = W_ + W_.transpose().eval();

    Eigen::VectorXd every_row_sum = W_.rowwise().sum();
    Eigen::MatrixXd D = every_row_sum.asDiagonal();
    Eigen::MatrixXd L = D - W_;

    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(L);
//    const Eigen::MatrixXd &vector_matrix = eigen_solver.pseudoEigenvectors();
    const Eigen::MatrixXd &vector_matrix = eigen_solver.eigenvectors().real();
    std::cout << "row: " << vector_matrix.rows() << std::endl;
    std::cout << "col: " << vector_matrix.rows() << std::endl;

    Eigen::MatrixXd V(source_points.size(), K_);
    for (int i = 0; i < K_; ++i) {
        V.col(i) = vector_matrix.col(source_points.size() - i - 1);
    }

    std::vector<Eigen::VectorXd> vs;
    for (unsigned int i = 0; i < V.rows(); ++i) {
        vs.emplace_back(V.row(i));
    }

    Kmeans kmeans(K_, 100, 0.0001);
    kmeans.Clustering(vs);
    std::vector<Kmeans::Cluster> kmeans_cluster = kmeans.GetCluster();

    clusters_.resize(K_);
    for (int i = 0; i < K_; ++i) {
        for (unsigned int j = 0; j < kmeans_cluster[i].indices_.size(); ++j) {
            unsigned int index = kmeans_cluster[i].indices_[j];
            clusters_[i].indices_.emplace_back(index);
            clusters_[i].points_.emplace_back(source_points[index]);
        }
    }

}

std::vector<SpectralClustering::Cluster> SpectralClustering::GetCluster() const {
    return clusters_;
}