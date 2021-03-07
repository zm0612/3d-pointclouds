//
// Created by Zhang Zhimeng on 2021/3/6.
//

#ifndef LECTURE_1_MODEL_DATA_H
#define LECTURE_1_MODEL_DATA_H

#include <eigen3/Eigen/Core>
#include <vector>

class ModelData{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ModelData() = default;

    unsigned int vertex_number_ = 0;
    unsigned int face_number_ = 0;
    unsigned int edge_number_ = 0;

    typedef typename std::vector<Eigen::Vector3d,
            Eigen::aligned_allocator<Eigen::Vector3d>> TypeVertexVector;
    typedef typename std::vector<Eigen::Matrix<unsigned int, 4, 1>,
            Eigen::aligned_allocator<Eigen::Matrix<unsigned int, 4, 1>>> TypeFaceVector;

    TypeVertexVector vertices_;
    TypeFaceVector faces_;
};

#endif //LECTURE_1_MODEL_DATA_H