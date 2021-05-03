//
// Created by Zhang Zhimeng on 2021/3/6.
//

#include "model_net_tool.h"
#include "pcl_viewer.h"

#include <iostream>

int main(int argc, char** argv){
    ModelNetTool model_net_tool;

    ModelData model_data;
    std::string file_path="/home/meng/code_my/ModelNet40/airplane/test/airplane_0627.off";

    model_net_tool.ReadData(file_path, model_data);

    std::cout << "vertex number: " << model_data.vertex_number_ << std::endl;
    std::cout << "face number: " << model_data.face_number_ << std::endl;
    std::cout << "edge number: " << model_data.edge_number_ << std::endl;

//    for (unsigned int i = 0; i < model_data.vertices_.size(); ++i) {
//        std::cout << model_data.vertices_.at(i).transpose() << std::endl;
//    }
//
//    for (unsigned int i = 0; i < model_data.faces_.size(); ++i) {
//        std::cout << model_data.faces_.at(i).transpose() << std::endl;
//    }
    PCLViewer::DisplayPointCloud(model_data.vertices_);

    return 0;
}
