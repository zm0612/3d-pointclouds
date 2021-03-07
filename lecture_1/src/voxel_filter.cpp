//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "data_type.h"
#include "voxel_filter.h"
#include <limits>
#include <iostream>
#include <algorithm>

bool myCmp(const int & a, const int& b)
{
    return a < b;
}

VoxelFilter::VoxelFilter(const Eigen::Vector3d &voxel_grid_size) {
    voxel_grid_size_ = voxel_grid_size;
}

void VoxelFilter::InputPoints(const ModelData::TypeVertexVector &source_points) {
    source_points_ = source_points;
}

void VoxelFilter::FilterByCentroid(Vector3ds &target_points) {
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::min();

    for (int i = 0; i < source_points_.size(); ++i) {
        const Eigen::Vector3d &point = source_points_.at(i);

        if (point.x() < x_min) {
            x_min = point.x();
        }

        if (point.x() > x_max) {
            x_max = point.x();
        }

        if (point.y() < y_min) {
            y_min = point.y();
        }

        if (point.y() > y_max) {
            y_max = point.z();
        }

        if (point.z() < z_min) {
            z_min = point.z();
        }

        if (point.z() > z_max) {
            z_max = point.z();
        }
    }

    if (voxel_grid_size_.x() == 0 ||
        voxel_grid_size_.y() == 0 ||
        voxel_grid_size_.z() == 0) {
        std::cerr << "voxel grid size equal 0" << std::endl;
    }

    int D_x = std::ceil((x_max - x_min) / voxel_grid_size_.x());
    int D_y = std::ceil((y_max - y_min) / voxel_grid_size_.y());
    int D_z = std::ceil((z_max - z_min) / voxel_grid_size_.z());

    std::vector<unsigned int> points_index(source_points_.size());
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        const Eigen::Vector3d &point = source_points_.at(i);
        unsigned int h_x = std::floor((point.x() - x_min) / voxel_grid_size_.x());
        unsigned int h_y = std::floor((point.y() - y_min) / voxel_grid_size_.y());
        unsigned int h_z = std::floor((point.z() - z_min) / voxel_grid_size_.z());

        unsigned h = h_x + h_y * D_x + h_z * D_x * D_y;

        points_index.at(i) = h;
    }

    std::stable_sort(points_index.begin(), points_index.end(), myCmp);

    for (int i = 0; i < points_index.size(); ++i) {
        std::cout << points_index.at(i) << std::endl;
    }
}