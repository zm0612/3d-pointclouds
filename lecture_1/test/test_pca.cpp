//
// Created by Zhang Zhimeng on 2021/3/6.
//
#include "principle_component_analysis.h"
#include "model_net_tool.h"
#include "pcl_viewer.h"

#include <chrono>

int main(){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0627.off";

    model_net_tool.ReadData(file_path, model_data);

    PrincipleComponentAnalysis principle_component_analysis;

    principle_component_analysis.InputData(model_data.vertices_);


    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    principle_component_analysis.CalculatePrincipleVector();

    Eigen::MatrixXd compressed_data = principle_component_analysis.Encoder(2);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout << "used time: " << time_used.count() << "second" << std::endl;

    PCLViewer::DisplayPointCloud(compressed_data);


    return 0;
}

