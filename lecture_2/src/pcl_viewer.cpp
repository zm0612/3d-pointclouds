//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "pcl_viewer.h"

void PCLViewer::DisplayPointCloud(const PointCloudPtr &point_cloud_ptr) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color(point_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud<Point>(point_cloud_ptr, single_color, "point cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point cloud");
    viewer.addCoordinateSystem (5.0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}

