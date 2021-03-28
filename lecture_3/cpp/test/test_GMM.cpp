//
// Created by Zhang Zhimeng on 2021/3/28.
//
#include "GMM.h"
#include "viewer.h"

#include <random>

int main(int argc, char** argv) {
    std::default_random_engine engine;
    std::normal_distribution<double> n_1(0, 1);
    std::normal_distribution<double> n_2(0, 2);

    Eigen::Vector3d center_0(0,0,0);
    Eigen::Vector3d center_1(5,5,0);

    std::vector<Eigen::Vector3d> points;

    for (int i = 0; i < 500; ++i) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        center.x() = center_0.x() + n_1(engine);
        center.y() = center_0.y() + n_2(engine);
        points.emplace_back(center);
        center.x() = center_1.y() + n_1(engine);
        center.y() = center_1.x() + n_2(engine);
        points.emplace_back(center);
    }

//    PCLViewer::DisplayPointCloud(points);

    std::vector<Eigen::VectorXd> points_2d;
    for (unsigned int i = 0; i < points.size(); ++i) {
        points_2d.push_back(points[i].block<2,1>(0,0));
    }

    GMM gmm(2, 10);

    gmm.Fit(points_2d);

    return 0;
}

