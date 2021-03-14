//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "pcl_viewer.h"
#include "data_type.h"
#include "model_net_tool.h"
#include "surface_normal_estimation.h"

#include <chrono>

int main(int argc, char** argv){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0672.off";

    model_net_tool.ReadData(file_path, model_data);

    Vector3ds normal_vectors;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    SurfaceNormalEstimation surface_normal_estimation;
    normal_vectors = surface_normal_estimation.CalculateNormalVector(model_data.vertices_, 5.0);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "used time: " << time_used.count() << "second" << std::endl;

    PCLViewer::DisplayNormalVector(model_data.vertices_, normal_vectors);

    return 0;
}
