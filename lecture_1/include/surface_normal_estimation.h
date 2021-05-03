//
// Created by Zhang Zhimeng on 2021/3/7.
//

#ifndef LECTURE_1_SURFACE_NORMAL_ESTIMATION_H
#define LECTURE_1_SURFACE_NORMAL_ESTIMATION_H

#include "data_type.h"
#include "model_data.h"

#include<eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

class SurfaceNormalEstimation{
public:
    SurfaceNormalEstimation() = default;

    Vector3ds CalculateNormalVector(const ModelData::TypeVertexVector& points, const double radius);

    Vector3ds CalculateNormalVector(const ModelData::TypeVertexVector& points, const int number_neighbor);
};

#endif //LECTURE_1_SURFACE_NORMAL_ESTIMATION_H
