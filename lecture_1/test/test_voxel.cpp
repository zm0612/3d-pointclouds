//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "voxel_filter.h"
#include "model_net_tool.h"

int main(){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0627.off";

    model_net_tool.ReadData(file_path, model_data);

    VoxelFilter voxel_filter(Eigen::Vector3d(5.0,5.0,5.0));
    voxel_filter.InputPoints(model_data.vertices_);

    Vector3ds filtered_points;
    voxel_filter.FilterByCentroid(filtered_points);

    return 0;
}

