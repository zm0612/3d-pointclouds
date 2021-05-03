//
// Created by meng on 2021/3/16.
//

#ifndef LECTURE_2_BRUTE_FORCE_SEARCH_H
#define LECTURE_2_BRUTE_FORCE_SEARCH_H

#include "data_type.h"
#include <eigen3/Eigen/Core>

class BruteForceSearch{
public:
    BruteForceSearch(const PointCloudPtr& point_cloud_ptr);

    std::vector<std::pair<unsigned int, float>> QueryPoints(const Eigen::Vector3f& query_point,
                                                            const unsigned int count = 1);

private:
    PointCloudPtr point_cloud_ptr_;
};

#endif //LECTURE_2_BRUTE_FORCE_SEARCH_H
