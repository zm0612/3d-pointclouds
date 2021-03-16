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

void PCLViewer::DisplayQueryResult(const PointCloudPtr &point_cloud_ptr,
                                   const Point &query_point,
                                   const PointCloudPtr &result_point_ptr) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color_0(point_cloud_ptr, 255, 255, 255);
    viewer.addPointCloud<Point>(point_cloud_ptr, single_color_0, "point cloud 0");

    PointCloudPtr query_point_cloud_ptr(new PointCloud);
    query_point_cloud_ptr->push_back(query_point);
    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color_1(query_point_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud<Point>(query_point_cloud_ptr, single_color_1, "point cloud 1");

    pcl::visualization::PointCloudColorHandlerCustom<Point> single_color_2(result_point_ptr, 0, 255, 0);
    viewer.addPointCloud<Point>(result_point_ptr, single_color_2, "point cloud 2");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud 0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "point cloud 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point cloud 2");
    viewer.addCoordinateSystem (5.0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}

