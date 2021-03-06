//
// Created by Zhang Zhimeng on 2021/3/22.
//

#ifndef CLUSTER_KMEANS_H
#define CLUSTER_KMEANS_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <random>

class Kmeans {
public:
    struct Cluster {
        Eigen::VectorXd center_;
        std::vector<Eigen::VectorXd> points_;
        std::vector<unsigned int> indices_;
    };

    Kmeans(unsigned int K, unsigned int max_iter, double distance_threshold);

    void Clustering(const std::vector<Eigen::VectorXd> &source_points);

    std::vector<Cluster> GetCluster() const;

    std::vector<Cluster> clusters_;

private:
    void InitKCenters();

    void ClearClusterPoint();

    double CalculateCenter();

private:
    unsigned int K_;
    unsigned int max_iter_;
    double distance_threshold_;
    std::vector<Eigen::VectorXd> source_points_;
};

#endif //CLUSTER_KMEANS_H
