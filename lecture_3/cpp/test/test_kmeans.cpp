//
// Created by Zhang Zhimeng on 2021/3/22.
//
#include "kmeans.h"
#include "viewer.h"

#include <eigen3/Eigen/Dense>
#include <iostream>

int main(int argc, char** argv){
    Eigen::Vector3d center_0(1,1,1);
    Eigen::Vector3d center_1(50,50,50);

    std::vector<Eigen::VectorXd> points;

    for (int i = 0; i < 100; ++i) {
        center_0 = center_0 + Eigen::Vector3d::Random() * 3;
//        center_0.z() = 0.0;
        points.emplace_back(center_0);

        center_1 = center_1 + Eigen::Vector3d::Random() * 3;
//        center_1.z() = 0.0;
        points.emplace_back(center_1);
    }

    Kmeans kmeans(2, 300, 0.000001);

    kmeans.Clustering(points);

    PCLViewer::DisplayPointCloud(kmeans.GetCluster());

    return 0;
}

