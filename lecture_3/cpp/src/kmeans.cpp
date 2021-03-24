//
// Created by Zhang Zhimeng on 2021/3/22.
//
#include "kmeans.h"
#include "viewer.h"
#include <iostream>

Kmeans::Kmeans(unsigned int K, unsigned int max_iter, double distance_threshold)
        :K_(K), max_iter_(max_iter), distance_threshold_(distance_threshold){
    clusters_.resize(K_);
}

std::vector<Kmeans::Cluster> Kmeans::GetCluster() const{
    return clusters_;
}

void Kmeans::InitKCenters() {
    unsigned int number_point = source_points_.size();

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, number_point-1);

    for (unsigned int i = 0; i < K_; ++i) {
        Eigen::VectorXd center = source_points_[distribution(rng)];
        clusters_.at(i).center_ = center;
    }
}

void Kmeans::ClearClusterPoint() {
    for (unsigned int i = 0; i < K_; ++i) {
        clusters_[i].points_.clear();
    }
}

double Kmeans::CalculateCenter() {
    double delta_dist = 0.0;

    for (unsigned int i = 0; i < K_; ++i) {
        Eigen::VectorXd center;
        center.resize(clusters_[0].points_[0].rows());
        center.setZero();
        for (const auto & point : clusters_[i].points_) {
            center += point;
        }
        Eigen::VectorXd new_center = center / static_cast<double>(clusters_[i].points_.size());
        delta_dist += (clusters_[i].center_ - new_center).norm();
        clusters_[i].center_ = new_center;
    }

    return delta_dist;
}

void Kmeans::Clustering(const std::vector<Eigen::VectorXd> &source_points) {
    source_points_ = source_points;
    InitKCenters();

    for (unsigned int i = 0; i < max_iter_; ++i) {
        ClearClusterPoint();

        for (unsigned int j = 0; j < source_points_.size(); ++j) {

            Eigen::VectorXd point = source_points_[j];
            double min_dist = std::numeric_limits<double>::max();
            int cluster_index = -1;
            for (int k = 0; k < K_; ++k) {
                double dist = (point - clusters_[k].center_).norm();
                if (dist < min_dist){
                    min_dist = dist;
                    cluster_index = k;
                }
            }

            clusters_[cluster_index].points_.emplace_back(point);
        }

        double delta_dist = CalculateCenter();
        std::cout << "iteration: " << i << "  delta distance: " << delta_dist << std::endl;

        if (delta_dist < distance_threshold_){
            std::cout << "delta distance less than threshold: " << distance_threshold_ << std::endl;
            break;
        }
    }
}