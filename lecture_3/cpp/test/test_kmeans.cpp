//
// Created by Zhang Zhimeng on 2021/3/22.
//
#include "kmeans.h"
#include "viewer.h"
#include "tic_toc.h"

int main(int argc, char** argv){
    Eigen::Vector3d center_0(1,1,1);
    Eigen::Vector3d center_1(50,50,50);
    Eigen::Vector3d center_2(10,10,10);
    Eigen::Vector3d center_3(30,30,30);

    std::vector<Eigen::VectorXd> points;

    for (int i = 0; i < 500; ++i) {
        center_0 = center_0 + Eigen::Vector3d::Random() * 3;
        points.emplace_back(center_0);
        center_1 = center_1 + Eigen::Vector3d::Random() * 3;
        points.emplace_back(center_1);
        center_2 = center_2 + Eigen::Vector3d::Random() * 3;
        points.emplace_back(center_2);
        center_3 = center_3 + Eigen::Vector3d::Random() * 3;
        points.emplace_back(center_3);
    }

    Kmeans kmeans(20, 300, 0.000001);

    TicToc tic_toc(true);
    kmeans.Clustering(points);
    tic_toc.toc("kmeans");

    PCLViewer::DisplayPointCloud(kmeans.GetCluster());

    return 0;
}

