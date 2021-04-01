//
// Created by Zhang Zhimeng on 2021/3/31.
//

#ifndef CLUSTER_SPECTRAL_CLUSTERING_H
#define CLUSTER_SPECTRAL_CLUSTERING_H

#include <eigen3/Eigen/Dense>

class SpectralClustering{
public:
    SpectralClustering();

private:
    Eigen::MatrixXd W_;
};

#endif //CLUSTER_SPECTRAL_CLUSTERING_H
