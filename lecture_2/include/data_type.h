//
// Created by Zhang Zhimeng on 2021/3/13.
//

#ifndef LECTURE_2_DATA_TYPE_H
#define LECTURE_2_DATA_TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;

#endif //LECTURE_2_DATA_TYPE_H
