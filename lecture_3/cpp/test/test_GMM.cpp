//
// Created by Zhang Zhimeng on 2021/3/28.
//
#include "GMM.h"
#include "viewer.h"
#include "tic_toc.h"
#include "generate_data.h"

#include <random>

int main(int argc, char **argv) {

    std::vector<Eigen::Vector3d> points;

    points = DataGenerator::GenerateNormalDistribution();
//    points = DataGenerator::GenerateCircleDistribution();

    std::vector<Eigen::VectorXd> points_2d;
    for (unsigned int i = 0; i < points.size(); ++i) {
        points_2d.emplace_back(points[i].block<2, 1>(0, 0));
    }

    GMM gmm(3, 100, 0.0001);

    TicToc tic_toc(1);
    gmm.Fit(points_2d);
    tic_toc.toc("GMM cluster");

    PCLViewer::DisplayPointCloud(gmm.GetClusters());

    return 0;
}

