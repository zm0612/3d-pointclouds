//
// Created by meng on 2021/3/29.
//

#ifndef CLUSTER_GENERATE_DATA_H
#define CLUSTER_GENERATE_DATA_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>

class DataGenerator{
public:
    DataGenerator() = default;

    static std::vector<Eigen::Vector3d> GenerateNormalDistribution();
    static std::vector<Eigen::Vector3d> GenerateCircleDistribution();
};

#endif //CLUSTER_GENERATE_DATA_H
