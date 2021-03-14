//
// Created by Zhang Zhimeng on 2021/3/13.
//

#ifndef LECTURE_2_READ_DATA_H
#define LECTURE_2_READ_DATA_H

#include "data_type.h"

class ReadData{
public:
    ReadData() = default;

    static bool Read(const std::string& point_file_path, PointCloudPtr& point_cloud_ptr);
};

#endif //LECTURE_2_READ_DATA_H
