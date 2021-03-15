//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "read_data.h"
#include "kdtree.h"

#include <chrono>

int main(int argc, char** argv){
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::string point_file_path = "../data/000000.bin";
    ReadData::Read(point_file_path, point_cloud_ptr);

    KDTree kd_tree;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    kd_tree.BuildTree(point_cloud_ptr, 10);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "build kdtree use time: " << time_used.count() * 1000 << " ms" << std::endl;

    return 0;
}
