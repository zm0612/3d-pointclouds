//
// Created by Zhang Zhimeng on 2021/3/6.
//
#include "principle_component_analysis.h"
#include "model_net_tool.h"
#include "pcl_viewer.h"

int main(){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0627.off";

    model_net_tool.ReadData(file_path, model_data);

    PrincipleComponentAnalysis principle_component_analysis;

    principle_component_analysis.InputData(model_data.vertices_);

    principle_component_analysis.CalculatePrincipleVector();

    Eigen::MatrixXd compressed_data = principle_component_analysis.Encoder(2);

    PCLViewer::DisplayPointCloud(compressed_data);

    return 0;
}

