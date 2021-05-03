//
// Created by Zhang Zhimeng on 2021/3/6.
//

#ifndef LECTURE_1_MODEL_DATA_H
#define LECTURE_1_MODEL_DATA_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

    static pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointCloud(const TypeVertexVector& points){
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

        for (int i = 0; i < points.size(); ++i) {
            pcl::PointXYZ point_xyz;
            point_xyz.x = static_cast<float>(points.at(i).x());
            point_xyz.y = static_cast<float>(points.at(i).y());
            point_xyz.z = static_cast<float>(points.at(i).z());

            point_cloud_ptr->push_back(point_xyz);
        }

        return point_cloud_ptr;
    }

    TypeVertexVector vertices_;
    TypeFaceVector faces_;
};

#endif //LECTURE_1_MODEL_DATA_H