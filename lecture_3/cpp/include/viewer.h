//
// Created by Zhang Zhimeng on 2021/3/23.
//

#ifndef CLUSTER_VIEWER_H
#define CLUSTER_VIEWER_H

#include "kmeans.h"
#include <pcl/visualization/cloud_viewer.h>

class PCLViewer{
public:
    PCLViewer() = default;

    static void DisplayPointCloud(std::vector<Kmeans::Cluster> clusters);

    static void DisplayPointCloud(std::vector<Eigen::Vector3d> points);
};

#endif //CLUSTER_VIEWER_H
