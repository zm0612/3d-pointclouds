//
// Created by Zhang Zhimeng on 2021/3/6.
//
#include "pcl_viewer.h"

#include <pcl/visualization/cloud_viewer.h>

void PCLViewer::DisplayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud_ptr) {
    pcl::visualization::PCLVisualizer viewer("3d viewer");

    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(point_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(point_cloud_ptr, single_color, "point cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
    viewer.addCoordinateSystem (5.0);
    viewer.initCameraParameters ();

    while (!viewer.wasStopped()){
        viewer.spin();
    }
}

void PCLViewer::DisplayPointCloud(const ModelData::TypeVertexVector &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    double center_x = 0, center_y = 0, center_z = 0;
    for (unsigned int i = 0; i < points.size(); ++i) {
        center_x = center_x + points[i].x();
        center_y = center_y + points[i].y();
        center_z = center_z + points[i].z();
    }
    center_x = center_x / static_cast<double>(points.size());
    center_y = center_y / static_cast<double>(points.size());
    center_z = center_z / static_cast<double>(points.size());

    for (unsigned int i = 0; i < points.size(); ++i) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>((points.at(i).x() - center_x) / 500.0);
        pcl_point.y = static_cast<float>((points.at(i).y() - center_y) / 500.0);
        pcl_point.z = static_cast<float>((points.at(i).z() - center_z) / 500.0);
        cloud_ptr->push_back(pcl_point);
    }

    pcl::visualization::PCLVisualizer viewer("3d viewer");

    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_ptr, single_color, "point cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point cloud");
//    viewer.addCoordinateSystem (5.0);
    viewer.initCameraParameters ();

    while (!viewer.wasStopped()){
        viewer.spin();
    }
}

void PCLViewer::DisplayPointCloud(const Eigen::MatrixXd &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    double center_x = 0, center_y = 0, center_z = 0;
    for (unsigned int i = 0; i < points.cols(); ++i) {
        center_x = center_x + points.col(i).x();
        center_y = center_y + points.col(i).y();
        center_z = center_z + points.col(i).z();
    }
    center_x = center_x / static_cast<double>(points.size());
    center_y = center_y / static_cast<double>(points.size());
    center_z = center_z / static_cast<double>(points.size());

    for (unsigned int i = 0; i < points.cols(); ++i) {
        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>((points.col(i).x() - center_x) / 500.0);
        pcl_point.y = static_cast<float>((points.col(i).y() - center_y) / 500.0);
        pcl_point.z = static_cast<float>((points.col(i).z() - center_z) / 500.0);
        cloud_ptr->push_back(pcl_point);
    }

    pcl::visualization::CloudViewer viewer("point cloud");
    viewer.showCloud(cloud_ptr);

    while (!viewer.wasStopped()){}
}

void PCLViewer::DisplayNormalVector(const ModelData::TypeVertexVector &points, const Vector3ds &normal_vectors) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_clouds(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < points.size(); ++i) {
        pcl::Normal normal;
        pcl::PointXYZ point;
        normal.normal_x = static_cast<float>(normal_vectors.at(i).x());
        normal.normal_y = static_cast<float>(normal_vectors.at(i).y());
        normal.normal_z = static_cast<float>(normal_vectors.at(i).z());

        point.x = static_cast<float>(points[i].x());
        point.y = static_cast<float>(points[i].y());
        point.z = static_cast<float>(points[i].z());

        normals->push_back(normal);
        point_clouds->push_back(point);
    }

    pcl::visualization::PCLVisualizer viewer("Normal viewer");

    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(point_clouds, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(point_clouds, single_color, "point cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(point_clouds, normals, 1, 5, "normal");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point cloud");

    while (!viewer.wasStopped()){
        viewer.spin();
    }
}