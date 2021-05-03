//
// Created by Zhang Zhimeng on 2021/3/6.
//
#include "principle_component_analysis.h"

#include <iostream>

void PrincipleComponentAnalysis::InputData(const Eigen::Vector3d &point) {
    if (X_.cols() == 0) {
        X_.resize(3, 1);
        X_ = point;
    } else {
        X_.conservativeResize(3, X_.cols() + 1);
        X_.block<3, 1>(0, X_.cols() - 1) = point;
    }
}

void PrincipleComponentAnalysis::InputData(const ModelData::TypeVertexVector &points) {
    int number_points = points.size();

    X_.resize(3, number_points);

    for (int i = 0; i < number_points; ++i) {
        X_.block<3,1>(0, i) = points[i];
    }
}

void PrincipleComponentAnalysis::CalculatePrincipleVector() {
    Eigen::Vector3d center = X_.rowwise().mean();
    Eigen::MatrixXd normalized_X = X_.colwise() - center;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized_X, Eigen::ComputeFullU);
    principle_vector_ = svd.matrixU();
}

Eigen::Vector3d PrincipleComponentAnalysis::CalculateNormalVector() {
    Eigen::Vector3d center = X_.rowwise().mean();
    Eigen::MatrixXd normalized_X = X_.colwise() - center;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized_X, Eigen::ComputeFullU);

    return svd.matrixU().col(2);
}

Eigen::MatrixXd PrincipleComponentAnalysis::Encoder(unsigned int dim) {
    if (dim > principle_vector_.cols()){
        std::cerr << "dimension greater than number of principle vector!" << std::endl;
    }

    Eigen::MatrixXd compressed_X;
    compressed_X.resize(3, X_.cols());
    compressed_X.setZero();

    Eigen::Matrix3d part_principle_vector = Eigen::Matrix3d::Zero();

    for (int i = 0; i < dim; ++i) {
        part_principle_vector.row(i) = principle_vector_.col(i).transpose();
    }

    return part_principle_vector * X_;
}
