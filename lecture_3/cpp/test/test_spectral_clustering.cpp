//
// Created by Zhang Zhimeng on 2021/3/31.
//

#ifndef CLUSTER_TEST_SPECTRAL_CLUSTERING_H
#define CLUSTER_TEST_SPECTRAL_CLUSTERING_H

#include "spectral_clustering.h"
#include "generate_data.h"
#include "viewer.h"
#include "tic_toc.h"

int main(int argc, char** argv){
    std::vector<Eigen::Vector3d> points;
    points = DataGenerator::GenerateNormalDistribution();
//    points = DataGenerator::GenerateCircleDistribution();

    std::vector<Eigen::VectorXd> points_2d;
    for (unsigned int i = 0; i < points.size(); ++i) {
        points_2d.emplace_back(points[i].block<2, 1>(0, 0));
    }

    SpectralClustering spectral_clustering(3);

    TicToc tic_toc(1);
    spectral_clustering.Fit(points_2d);
    tic_toc.toc("spctral_clustering");

    PCLViewer::DisplayPointCloud(spectral_clustering.GetCluster());

    return 0;
}

#endif //CLUSTER_TEST_SPECTRAL_CLUSTERING_H
