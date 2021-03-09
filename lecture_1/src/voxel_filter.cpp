//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "data_type.h"
#include "voxel_filter.h"
#include <limits>
#include <iostream>
#include <algorithm>

bool myCmp(const VoxelFilter::SearchIndex& a, const VoxelFilter::SearchIndex& b)
{
    return a.voxel_index < b.voxel_index;
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
            y_max = point.y();
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

    std::vector<SearchIndex> search_indices(source_points_.size());
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        const Eigen::Vector3d &point = source_points_.at(i);
        unsigned int h_x = std::floor((point.x() - x_min) / voxel_grid_size_.x());
        unsigned int h_y = std::floor((point.y() - y_min) / voxel_grid_size_.y());
        unsigned int h_z = std::floor((point.z() - z_min) / voxel_grid_size_.z());

        unsigned h = h_x + h_y * D_x + h_z * D_x * D_y;

        search_indices.at(i).point_index = i;
        search_indices.at(i).voxel_index = h;
    }

    std::stable_sort(search_indices.begin(), search_indices.end(), myCmp);

    Eigen::Vector3d point(0,0,0);
    int count = 0;
    for (unsigned int i = 1; i < search_indices.size(); ++i) {
        point += source_points_.at(search_indices.at(i-1).point_index);
        count++;

        if (search_indices.at(i - 1).voxel_index != search_indices.at(i).voxel_index ) {
            point = point / (count * 1.0);
            target_points.emplace_back(point);

            if (i == search_indices.size() - 1){
                target_points.emplace_back(source_points_.at(search_indices.at(i).point_index));
                break;
            }

            count = 0;
            point.setZero();
        }

        if (search_indices.at(i-1).voxel_index == search_indices.at(i).voxel_index
            && i == search_indices.size())
        {
            target_points.back() = (target_points.back() + source_points_.at(search_indices.at(i).point_index)) / 2.0;
        }
    }
}

void VoxelFilter::FilterByRandom(Vector3ds &target_points) {
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
            y_max = point.y();
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

    std::vector<SearchIndex> search_indices(source_points_.size());
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        const Eigen::Vector3d &point = source_points_.at(i);
        unsigned int h_x = std::floor((point.x() - x_min) / voxel_grid_size_.x());
        unsigned int h_y = std::floor((point.y() - y_min) / voxel_grid_size_.y());
        unsigned int h_z = std::floor((point.z() - z_min) / voxel_grid_size_.z());

        unsigned h = h_x + h_y * D_x + h_z * D_x * D_y;

        search_indices.at(i).point_index = i;
        search_indices.at(i).voxel_index = h;
    }

    std::stable_sort(search_indices.begin(), search_indices.end(), myCmp);

    Eigen::Vector3d point(0,0,0);
    int count = 0;
    for (unsigned int i = 1; i < search_indices.size(); ++i) {
        point += source_points_.at(search_indices.at(i-1).point_index);
        count++;

        if (search_indices.at(i - 1).voxel_index != search_indices.at(i).voxel_index ) {
            point = point / (count * 1.0);
            target_points.emplace_back(point);

            if (i == search_indices.size() - 1){
                target_points.emplace_back(source_points_.at(search_indices.at(i).point_index));
                break;
            }

            count = 0;
            point.setZero();
        }

        if (search_indices.at(i-1).voxel_index == search_indices.at(i).voxel_index
            && i == search_indices.size())
        {
            target_points.back() = (target_points.back() + source_points_.at(search_indices.at(i).point_index)) / 2.0;
        }
    }
}