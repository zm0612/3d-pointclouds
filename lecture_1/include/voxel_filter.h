//
// Created by Zhang Zhimeng on 2021/3/7.
//

#ifndef LECTURE_1_VOXEL_FILTER_H
#define LECTURE_1_VOXEL_FILTER_H

#include "data_type.h"
#include "model_data.h"

class VoxelFilter{
public:
    VoxelFilter(const Eigen::Vector3d& voxel_grid_size);

    void InputPoints(const ModelData::TypeVertexVector& source_points);

    void FilterByCentroid(Vector3ds& target_points);
    void FilterByRandom(Vector3ds& target_points);

private:
    ModelData::TypeVertexVector source_points_;
    ModelData::TypeVertexVector target_points_;

    Eigen::Vector3d voxel_grid_size_;
};

#endif //LECTURE_1_VOXEL_FILTER_H
