//
// Created by Zhang Zhimeng on 2021/3/6.
//

#ifndef LECTURE_1_PRINCIPLE_COMPONENT_ANALYSIS_H
#define LECTURE_1_PRINCIPLE_COMPONENT_ANALYSIS_H

#include "model_data.h"

#include <eigen3/Eigen/Dense>
#include <vector>

class PrincipleComponentAnalysis{
public:
    PrincipleComponentAnalysis() = default;

    void InputData(const Eigen::Vector3d& point);

    void InputData(const ModelData::TypeVertexVector& points);

    void CalculatePrincipleVector();

    Eigen::Vector3d CalculateNormalVector();

    Eigen::MatrixXd Encoder(unsigned int dim);

private:

    Eigen::Matrix<double, 3, Eigen::Dynamic> X_;
    Eigen::Matrix<double, 3, 3> principle_vector_;
};

#endif //LECTURE_1_PRINCIPLE_COMPONENT_ANALYSIS_H
