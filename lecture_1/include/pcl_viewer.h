//
// Created by Zhang Zhimeng on 2021/3/6.
//

#ifndef LECTURE_1_PCL_VIEWER_H
#define LECTURE_1_PCL_VIEWER_H

#include "data_type.h"
#include "model_data.h"

class PCLViewer{
public:
    PCLViewer() = default;

    static void DisplayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_ptr);
    static void DisplayPointCloud(const ModelData::TypeVertexVector& points);
    static void DisplayPointCloud(const Eigen::MatrixXd& points);
    static void DisplayNormalVector(const ModelData::TypeVertexVector& points,
                                    const Vector3ds& normal_vectors);
};

#endif //LECTURE_1_PCL_VIEWER_H
