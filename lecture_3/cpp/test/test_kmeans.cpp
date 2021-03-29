//
// Created by Zhang Zhimeng on 2021/3/22.
//
#include "kmeans.h"
#include "viewer.h"
#include "tic_toc.h"
#include "generate_data.h"

int main(int argc, char** argv){
    std::vector<Eigen::Vector3d> points;
    points = DataGenerator::GenerateNormalDistribution();
//    points = DataGenerator::GenerateCircleDistribution();

    std::vector<Eigen::VectorXd> points_2d;
    for (unsigned int i = 0; i < points.size(); ++i) {
        points_2d.emplace_back(points[i].block<3, 1>(0, 0));
    }

    Kmeans kmeans(3u, 300u, 0.000001);

    TicToc tic_toc(true);
    kmeans.Clustering(points_2d);
    tic_toc.toc("kmeans");

    PCLViewer::DisplayPointCloud(kmeans.GetCluster());

    return 0;
}

