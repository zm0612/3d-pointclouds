//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "pcl_viewer.h"
#include "data_type.h"
#include "model_net_tool.h"
#include "surface_normal_estimation.h"

int main(int argc, char** argv){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/cone/test/cone_0168.off";

    model_net_tool.ReadData(file_path, model_data);

    Vector3ds normal_vectors;

    SurfaceNormalEstimation surface_normal_estimation;
    normal_vectors = surface_normal_estimation.CalculateNormalVector(model_data.vertices_, 6.0);

    PCLViewer::DisplayNormalVector(model_data.vertices_, normal_vectors);

    return 0;
}
