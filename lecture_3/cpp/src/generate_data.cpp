//
// Created by meng on 2021/3/29.
//
#include "generate_data.h"

std::vector<Eigen::Vector3d> DataGenerator::GenerateCircleDistribution() {
    std::default_random_engine engine;
    std::normal_distribution<double> n_1(0, 1);
    std::normal_distribution<double> n_2(0, 1);

    Eigen::Vector3d center_0(0,0,0);
    Eigen::Vector3d center_1(5,5,0);
    Eigen::Vector3d center_2(10,10,0);

    std::vector<Eigen::Vector3d> points;

    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        center.x() = center_0.x() + n_1(engine);
        center.y() = center_0.y() + n_2(engine);
        points.emplace_back(center);
        center.x() = center_1.y() + n_1(engine);
        center.y() = center_1.x() + n_2(engine);
        points.emplace_back(center);
        center.x() = center_2.y() + n_1(engine);
        center.y() = center_2.x() + n_2(engine);
        points.emplace_back(center);
    }

    return points;
}

std::vector<Eigen::Vector3d> DataGenerator::GenerateNormalDistribution() {
    std::default_random_engine engine;
    std::normal_distribution<double> n_1(0, 1);
    std::normal_distribution<double> n_2(0, 4);

    Eigen::Vector3d center_0(0,0,0);
    Eigen::Vector3d center_1(5,5,0);
    Eigen::Vector3d center_2(10,10,0);

    std::vector<Eigen::Vector3d> points;

    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        center.x() = center_0.x() + n_1(engine);
        center.y() = center_0.y() + n_2(engine);
        points.emplace_back(center);
        center.x() = center_1.y() + n_1(engine);
        center.y() = center_1.x() + n_2(engine);
        points.emplace_back(center);
        center.x() = center_2.y() + n_1(engine);
        center.y() = center_2.x() + n_2(engine);
        points.emplace_back(center);
    }

    return points;
}

