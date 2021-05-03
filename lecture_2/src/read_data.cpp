//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "read_data.h"

#include <fstream>

bool ReadData::Read(const std::string &point_file_path, PointCloudPtr& point_cloud_ptr) {
    FILE *stream;

    int32_t num = 1000000;
    float* data = (float *) malloc(num*sizeof(float));

    float *x_ptr = data + 0;
    float *y_ptr = data + 1;
    float *z_ptr = data + 2;
    float *i_ptr = data + 3;

    stream = fopen(point_file_path.c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    for (int32_t i = 0; i < num; ++i) {
        Point point;
        point.x = *x_ptr;
        point.y = *y_ptr;
        point.z = *z_ptr;
        point.intensity = *i_ptr;

        point_cloud_ptr->push_back(point);

        x_ptr += 4;
        y_ptr += 4;
        z_ptr += 4;
        i_ptr += 4;
    }

    fclose(stream);
    free(data);

    std::cout << "totally read points: " << point_cloud_ptr->size() << std::endl;

    return true;
}

