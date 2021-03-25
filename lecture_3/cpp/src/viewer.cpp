//
// Created by Zhang Zhimeng on 2021/3/23.
//
#include "viewer.h"

void PCLViewer::DisplayPointCloud(std::vector<Kmeans::Cluster> clusters) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned int i = 0; i < clusters.size(); ++i) {
        for (unsigned int j = 0; j < clusters[i].points_.size(); ++j) {
            pcl::PointXYZ point_xyz;
            point_xyz.x = clusters[i].points_[j].x();
            point_xyz.y = clusters[i].points_[j].y();
            point_xyz.z = clusters[i].points_[j].z();

            source_points_ptr->push_back(point_xyz);
        }

        pcl::PointXYZ center_point;
        center_point.x = clusters[i].center_.x();
        center_point.y = clusters[i].center_.y();
        center_point.z = clusters[i].center_.z();
        center_points_ptr->push_back(center_point);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_0(source_points_ptr, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(source_points_ptr, 255, 0, 0);

    viewer.addPointCloud<pcl::PointXYZ>(source_points_ptr, single_color_0, "point cloud 0");
    viewer.addPointCloud<pcl::PointXYZ>(center_points_ptr, single_color_1, "point cloud 1");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point cloud 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 15, "point cloud 1");
    viewer.initCameraParameters();

    while (!viewer.wasStopped()){
        viewer.spin();
    }
}
