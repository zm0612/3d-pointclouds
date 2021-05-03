//
// Created by Zhang Zhimeng on 2021/3/31.
//
#include "spectral_clustering.h"
#include "kmeans.h"
#include <iostream>
#include "viewer.h"
#include <pcl/kdtree/kdtree_flann.h>

SpectralClustering::SpectralClustering(int K)
        : K_(K) {}

void SpectralClustering::Fit(const std::vector<Eigen::VectorXd> &source_points) {
    W_.resize(source_points.size(), source_points.size());
//    CalcuAdjacencyMatrix(source_points);
    for (unsigned int i = 0; i < source_points.size(); ++i) {
        for (unsigned int j = i; j < source_points.size(); ++j) {
            if (i == j) {
                W_(i, j) = 0;
            } else {
//                W_(i, j) = 1.0 / (source_points[i] - source_points[j]).squaredNorm();
                W_(i, j) = std::exp(-(source_points[i] - source_points[j]).squaredNorm());
                W_(j, i) = W_(i, j);
            }
        }
    }

    Eigen::VectorXd every_row_sum = W_.rowwise().sum();
    Eigen::MatrixXd D = every_row_sum.asDiagonal();
    Eigen::MatrixXd L = D - W_;
    Eigen::MatrixXd L_ew = D.inverse() * L;

    Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(L_ew);
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

    std::stable_sort(value_and_vector.begin(), value_and_vector.end(),
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

void SpectralClustering::CalcuAdjacencyMatrix(std::vector<Eigen::VectorXd> points) {
    W_.setZero();
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < points.size(); ++i) {
        pcl::PointXYZ point_xyz;
        point_xyz.x = points[i].x();
        point_xyz.y = points[i].y();
        if (points[i].rows() == 3) {
            point_xyz.z = points[i].z();
        } else{
            point_xyz.z = 0;
        }
        point_cloud->push_back(point_xyz);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann;
    kd_tree_flann.setInputCloud(point_cloud);

    int num_neighbors = 3;
    for (int i = 0; i < points.size(); ++i) {
        std::vector<int> indices(num_neighbors);
        std::vector<float> dists(num_neighbors);
        kd_tree_flann.nearestKSearch(point_cloud->points[i], num_neighbors, indices, dists);

        for (int j = 0; j < num_neighbors; ++j) {
            int neighbor_index = indices[j];
            if (i >= neighbor_index)
                continue;
            std::cout << dists[j] << std::endl;
            double weight = 1./dists[j];
            W_(i, neighbor_index) = weight;
            W_(neighbor_index, i) = weight;
        }
    }
}