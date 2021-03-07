//
// Created by Zhang Zhimeng on 2021/3/7.
//

#ifndef LECTURE_1_DATA_TYPE_H
#define LECTURE_1_DATA_TYPE_H

#include <eigen3/Eigen/Dense>
#include <vector>

typedef typename std::vector<Eigen::Vector3d,
        Eigen::aligned_allocator<Eigen::Vector3d>> Vector3ds;

#endif //LECTURE_1_DATA_TYPE_H
