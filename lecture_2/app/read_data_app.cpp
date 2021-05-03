//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "read_data.h"
#include "pcl_viewer.h"

int main(int argc, char** argv){
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::string point_file_path = "../data/000000.bin";
    ReadData::Read(point_file_path, point_cloud_ptr);

    PCLViewer::DisplayPointCloud(point_cloud_ptr);

    return 0;
}