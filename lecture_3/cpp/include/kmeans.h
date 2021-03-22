//
// Created by Zhang Zhimeng on 2021/3/22.
//

#ifndef CLUSTER_KMEANS_H
#define CLUSTER_KMEANS_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <random>

class Kmeans{
public:
    struct Cluster{
        Eigen::VectorXd center_;
        Eigen::MatrixXd points_;
    };

    Kmeans(unsigned int K);

    void Clustering(const Eigen::MatrixXd& source_points);

    std::vector<Cluster> clusters_;

private:
    void InitKCenters();

private:
    unsigned int K_ = 0;
    Eigen::MatrixXd source_points_;
};

#endif //CLUSTER_KMEANS_H
