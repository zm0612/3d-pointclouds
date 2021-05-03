//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "voxel_filter.h"
#include "model_net_tool.h"
#include "pcl_viewer.h"

#include <pcl/filters/voxel_grid.h>

int main(){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0627.off";

    model_net_tool.ReadData(file_path, model_data);

    VoxelFilter voxel_filter(Eigen::Vector3d(30.0,30.0,30.0));
    voxel_filter.InputPoints(model_data.vertices_);

    Vector3ds filtered_points;
    voxel_filter.FilterByCentroid(filtered_points);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(30, 30, 30);
    voxel_grid.setInputCloud(ModelData::ToPointCloud(model_data.vertices_));
    voxel_grid.filter(*point_cloud_ptr);

    PCLViewer::DisplayPointCloud(filtered_points);
    PCLViewer::DisplayPointCloud(point_cloud_ptr);

    return 0;
}

