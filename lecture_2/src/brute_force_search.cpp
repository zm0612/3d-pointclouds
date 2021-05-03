//
// Created by meng on 2021/3/16.
//
#include "brute_force_search.h"

BruteForceSearch::BruteForceSearch(const PointCloudPtr& point_cloud_ptr)
        : point_cloud_ptr_(new PointCloud) {
    point_cloud_ptr_ = point_cloud_ptr;
}

std::vector<std::pair<unsigned int, float>> BruteForceSearch::QueryPoints(const Eigen::Vector3f& query_point,
                                                                          const unsigned int capacity) {
    float worst_dist = std::numeric_limits<float>::max();
    unsigned int count = 0;
    std::vector<std::pair<unsigned int, float>> result(capacity);
    for (unsigned int i = 0; i < capacity; ++i) {
        result[i] = (std::pair<unsigned int, float>(0, std::numeric_limits<float>::max()));
    }

    for (unsigned int i = 0; i < point_cloud_ptr_->size(); ++i) {
        Point point = point_cloud_ptr_->at(i);
        Eigen::Vector3f point_eigen;
        point_eigen.x() = point.x;
        point_eigen.y() = point.y;
        point_eigen.z() = point.z;

        float dist = (point_eigen - query_point).norm();

        if (dist > worst_dist){
            continue;
        }

        if (count < capacity){
            count++;
        }

        unsigned int temp = count - 1;

        while (temp > (unsigned int)0){
            if (result[temp-1].second > dist){
                result[temp] = result[temp-1];
                --temp;
            } else{
                break;
            }
        }

        result[temp].second = dist;
        result[temp].first = i;
        worst_dist = result[capacity-1].second;
    }

    return result;
}

