//
// Created by Zhang Zhimeng on 2021/3/23.
//
#include "viewer.h"

void PCLViewer::DisplayPointCloud(std::vector<GMM::Cluster> clusters) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    for (unsigned int i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int j = 0; j < clusters.at(i).points_.size(); ++j) {
            pcl::PointXYZ point_xyz;

            point_xyz.x = clusters[i].points_[j].x();
            point_xyz.y = clusters[i].points_[j].y();

            if (clusters[i].points_[j].rows() == 2) {
                point_xyz.z = 0;
            } else {
                point_xyz.z = clusters[i].points_[j].z();
            }
            point_cloud_ptr->push_back(point_xyz);
        }

        double color_r = 0.0;
        double color_g = 0.0;
        double color_b = 0.0;
        if (clusters.size() <= 3) {
            if (i == 0) {
                color_r = 255.0;
                color_g = 0.0;
                color_b = 0.0;
            } else if (i == 1) {
                color_r = 0.0;
                color_g = 255.0;
                color_b = 0.0;
            } else if (i == 2) {
                color_r = 0.0;
                color_g = 0.0;
                color_b = 255.0;
            }
        } else {
            color_r = color_g = color_b = 255.0 / clusters.size() * i;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(point_cloud_ptr, color_r, color_g,
                                                                                     color_b);
        viewer.addPointCloud<pcl::PointXYZ>(point_cloud_ptr, single_color, "point cloud " + std::to_string(i));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                "point cloud " + std::to_string(i));
    }

    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}

void PCLViewer::DisplayPointCloud(std::vector<Kmeans::Cluster> clusters) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr center_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned int i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        for (unsigned int j = 0; j < clusters[i].points_.size(); ++j) {
            pcl::PointXYZ point_xyz;
            point_xyz.x = clusters[i].points_[j].x();
            point_xyz.y = clusters[i].points_[j].y();
            point_xyz.z = clusters[i].points_[j].z();

            source_points_ptr->push_back(point_xyz);
        }

//        pcl::PointXYZ center_point;
//        center_point.x = clusters[i].center_.x();
//        center_point.y = clusters[i].center_.y();
//        center_point.z = clusters[i].center_.z();
//        center_points_ptr->push_back(center_point);

        double color_r = 0.0;
        double color_g = 0.0;
        double color_b = 0.0;
        if (clusters.size() <= 3) {
            if (i == 0) {
                color_r = 255.0;
                color_g = 0.0;
                color_b = 0.0;
            } else if (i == 1) {
                color_r = 0.0;
                color_g = 255.0;
                color_b = 0.0;
            } else if (i == 2) {
                color_r = 0.0;
                color_g = 0.0;
                color_b = 255.0;
            }
        } else {
            color_r = color_g = color_b = 255.0 / clusters.size() * i;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(source_points_ptr, color_r,
                                                                                     color_g, color_b);
        viewer.addPointCloud<pcl::PointXYZ>(source_points_ptr, single_color, "point cloud " + std::to_string(i));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                "point cloud " + std::to_string(i));
    }

    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}

void PCLViewer::DisplayPointCloud(std::vector<Eigen::Vector3d> points) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_points_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned int i = 0; i < points.size(); ++i) {
        pcl::PointXYZ point_xyz;
        point_xyz.x = points[i].x();
        point_xyz.y = points[i].y();
        point_xyz.z = points[i].z();

        source_points_ptr->push_back(point_xyz);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_0(source_points_ptr, 255, 255, 255);

    viewer.addPointCloud<pcl::PointXYZ>(source_points_ptr, single_color_0, "point cloud 0");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point cloud 0");
    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}

void PCLViewer::DisplayPointCloud(std::vector<SpectralClustering::Cluster> clusters) {
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    for (unsigned int i = 0; i < clusters.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (unsigned int j = 0; j < clusters.at(i).points_.size(); ++j) {
            pcl::PointXYZ point_xyz;

            point_xyz.x = clusters[i].points_[j].x();
            point_xyz.y = clusters[i].points_[j].y();

            if (clusters[i].points_[j].rows() == 2) {
                point_xyz.z = 0;
            } else {
                point_xyz.z = clusters[i].points_[j].z();
            }
            point_cloud_ptr->push_back(point_xyz);
        }

        double color_r = 0.0;
        double color_g = 0.0;
        double color_b = 0.0;
        if (clusters.size() <= 3) {
            if (i == 0) {
                color_r = 255.0;
                color_g = 0.0;
                color_b = 0.0;
            } else if (i == 1) {
                color_r = 0.0;
                color_g = 255.0;
                color_b = 0.0;
            } else if (i == 2) {
                color_r = 0.0;
                color_g = 0.0;
                color_b = 255.0;
            }
        } else {
            color_r = color_g = color_b = 255.0 / clusters.size() * i;
        }
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(point_cloud_ptr, color_r, color_g,
                                                                                     color_b);
        viewer.addPointCloud<pcl::PointXYZ>(point_cloud_ptr, single_color, "point cloud " + std::to_string(i));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                "point cloud " + std::to_string(i));
    }

    viewer.initCameraParameters();

    while (!viewer.wasStopped()) {
        viewer.spin();
    }
}