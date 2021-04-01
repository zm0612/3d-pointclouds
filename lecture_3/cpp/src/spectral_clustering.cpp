//
// Created by Zhang Zhimeng on 2021/3/31.
//
#include "spectral_clustering.h"
#include "kmeans.h"
#include <iostream>
#include "viewer.h"

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
    const Eigen::VectorXd &value = eigen_solver.eigenvalues().real();
    const Eigen::MatrixXd &vector_matrix = eigen_solver.eigenvectors().real();

    Eigen::MatrixXd V;
    V = SortEigenVectorByValues(value, vector_matrix);

    std::vector<Eigen::VectorXd> vs;
    for (unsigned int i = 0; i < V.rows(); ++i) {
        vs.emplace_back(V.row(i));
    }

    Kmeans kmeans(K_, 100, 0.0001);
    kmeans.Clustering(vs);
    std::vector<Kmeans::Cluster> kmeans_cluster = kmeans.GetCluster();

//    PCLViewer::DisplayPointCloud(kmeans_cluster);

    clusters_.resize(K_);
    for (int i = 0; i < K_; ++i) {
        for (unsigned int j = 0; j < kmeans_cluster[i].indices_.size(); ++j) {
            unsigned int index = kmeans_cluster[i].indices_[j];
            clusters_[i].indices_.emplace_back(index);
            clusters_[i].points_.emplace_back(source_points[index]);
        }
    }
}

Eigen::MatrixXd SpectralClustering::SortEigenVectorByValues(const Eigen::VectorXd value,
                                                            const Eigen::MatrixXd &vector) {
    std::vector<std::pair<double, Eigen::VectorXd>> value_and_vector;
    int size = value.rows();
    value_and_vector.reserve(size);

    for (int i = 0; i < size; ++i) {
        value_and_vector.push_back({value[i], vector.col(i)});
    }

    std::sort(value_and_vector.begin(), value_and_vector.end(),
              [](const std::pair<double, Eigen::VectorXd> &pair1,
                 const std::pair<double, Eigen::VectorXd> &pair2) -> bool {
                  return pair1.first < pair2.first;
              });

    Eigen::MatrixXd matrix(vector.rows(), K_);
    for (int i = 0; i < K_; ++i) {
        matrix.col(i) = value_and_vector[i].second;
    }

    return matrix;
}

std::vector<SpectralClustering::Cluster> SpectralClustering::GetCluster() const {
    return clusters_;
}