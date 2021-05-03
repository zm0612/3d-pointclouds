//
// Created by Zhang Zhimeng on 2021/3/16.
//
#include "read_data.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <chrono>

int main(int argc, char** argv){
    PointCloudPtr point_cloud_ptr(new PointCloud);

    std::string point_file_path = "../data/000000.bin";
    ReadData::Read(point_file_path, point_cloud_ptr);

    Point point = point_cloud_ptr->at(1000);

    constexpr int K = 10000;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    pcl::KdTreeFLANN<Point> kd_tree_flann;
    std::chrono::steady_clock::time_point t0_build_tree = std::chrono::steady_clock::now();
    kd_tree_flann.setInputCloud(point_cloud_ptr);
    std::chrono::steady_clock::time_point t1_build_tree = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_build_tree = std::chrono::duration_cast<std::chrono::duration<double>>(t1_build_tree-t0_build_tree);
    std::cout << "build tree use time: " << time_used_build_tree.count() * 1000 << " ms" << std::endl;

    std::chrono::steady_clock::time_point t2_search = std::chrono::steady_clock::now();
    kd_tree_flann.nearestKSearch(point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    std::chrono::steady_clock::time_point t3_search = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_search = std::chrono::duration_cast<std::chrono::duration<double>>(t3_search-t2_search);
    std::cout << "query use time: " << time_used_search.count() * 1000 << " ms" << std::endl;

    constexpr float resolution = 1.0f;
    pcl::octree::OctreePointCloudSearch<Point> octree_point_cloud_search(resolution);
    std::chrono::steady_clock::time_point t4_search = std::chrono::steady_clock::now();
    octree_point_cloud_search.setInputCloud(point_cloud_ptr);
    octree_point_cloud_search.addPointsFromInputCloud();
    std::chrono::steady_clock::time_point t5_search = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_search_1 = std::chrono::duration_cast<std::chrono::duration<double>>(t5_search-t4_search);
    std::cout << "build octree use time: " << time_used_search_1.count() * 1000 << " ms" << std::endl;

    constexpr int K_1 = 100000;
    std::vector<int> pointIdxNKNSearch_1; //保存K近邻点的索引结果
    std::vector<float> pointNKNSquaredDistance_1;  //保存每个近邻点与查找点之间的欧式距离平方

    std::chrono::steady_clock::time_point t6_search = std::chrono::steady_clock::now();
    octree_point_cloud_search.nearestKSearch(point, K_1, pointIdxNKNSearch_1, pointNKNSquaredDistance_1);
    std::chrono::steady_clock::time_point t7_search = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_search_2 = std::chrono::duration_cast<std::chrono::duration<double>>(t7_search-t6_search);
    std::cout << "search octree use time: " << time_used_search_2.count() * 1000 << " ms" << std::endl;

    return 0;
}

