//
// Created by Zhang Zhimeng on 2021/3/22.
//
#include "kmeans.h"

Kmeans::Kmeans(unsigned int K):K_(K) {
    clusters_.resize(K_);
}

void Kmeans::InitKCenters() {
    unsigned int number_point = source_points_.cols();

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, number_point-1);

    for (unsigned int i = 0; i < K_; ++i) {
        Eigen::VectorXd center = source_points_.col(distribution(rng));
        clusters_.at(i).center_ = center;
    }
}

void Kmeans::Clustering(const Eigen::MatrixXd &source_points) {
    source_points_ = source_points;
}