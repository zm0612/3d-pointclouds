//
// Created by Zhang Zhimeng on 2021/3/31.
//

#ifndef CLUSTER_SPECTRAL_CLUSTERING_H
#define CLUSTER_SPECTRAL_CLUSTERING_H

#include <eigen3/Eigen/Dense>
#include <vector>

class SpectralClustering{
public:
    struct Cluster{
        std::vector<Eigen::VectorXd> points_;
        std::vector<unsigned int> indices_;
    };

    SpectralClustering(int K=-1);

    void Fit(const std::vector<Eigen::VectorXd>& source_points);

    std::vector<Cluster> GetCluster() const;

private:
    int K_;
    std::vector<Cluster> clusters_;
    Eigen::MatrixXd W_;
    std::vector<Eigen::VectorXd> source_points_;
};

#endif //CLUSTER_SPECTRAL_CLUSTERING_H
