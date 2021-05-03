//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "read_data.h"
#include "kdtree.h"
#include "brute_force_search.h"
#include "pcl_viewer.h"

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

    std::vector<unsigned int> query_indices;
    std::chrono::steady_clock::time_point t1_query = std::chrono::steady_clock::now();

    Point point = point_cloud_ptr->at(1000);
    Eigen::Vector3f point_eigen;
    point_eigen.x() = point.x;
    point_eigen.y() = point.y;
    point_eigen.z() = point.z;

    query_indices = kd_tree.QueryNearestNeighbor(10000, point_eigen);
    std::chrono::steady_clock::time_point t2_query = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_query = std::chrono::duration_cast<std::chrono::duration<double>>(t2_query-t1_query);
    std::cout << "query use time: " << time_used_query.count() * 1000 << " ms" << std::endl;

    PointCloudPtr result_point_cloud_ptr(new PointCloud);
    for (unsigned int i = 0; i < query_indices.size(); ++i) {
        result_point_cloud_ptr->push_back(point_cloud_ptr->at(query_indices[i]));
    }

    std::vector<std::pair<unsigned int, float>> result_brute_force_search;
    BruteForceSearch brute_force_search(point_cloud_ptr);
    std::chrono::steady_clock::time_point t3_bsf = std::chrono::steady_clock::now();
    result_brute_force_search = brute_force_search.QueryPoints(point_eigen, 10000);
    std::chrono::steady_clock::time_point t4_bsf = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_bsf = std::chrono::duration_cast<std::chrono::duration<double>>(t4_bsf-t3_bsf);
    std::cout << "bfs use time: " << time_used_bsf.count() * 1000 << " ms" << std::endl;

    PointCloudPtr result_bfs_point_cloud_ptr(new PointCloud);
    for (unsigned int i = 0; i < result_brute_force_search.size(); ++i) {
        result_bfs_point_cloud_ptr->push_back(point_cloud_ptr->at(result_brute_force_search[i].first));
    }

    PCLViewer::DisplayQueryResult(point_cloud_ptr, point, result_point_cloud_ptr);
//    PCLViewer::DisplayQueryResult(point_cloud_ptr, point, result_bfs_point_cloud_ptr);

    return 0;
}
