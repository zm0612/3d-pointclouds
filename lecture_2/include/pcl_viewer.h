//
// Created by Zhang Zhimeng on 2021/3/13.
//

#ifndef LECTURE_2_PCL_VIEWER_H
#define LECTURE_2_PCL_VIEWER_H

#include "data_type.h"

#include <pcl/visualization/cloud_viewer.h>

class PCLViewer{
public:

    PCLViewer() = default;

    static void DisplayPointCloud(const PointCloudPtr& point_cloud_ptr);
};

#endif //LECTURE_2_PCL_VIEWER_H
