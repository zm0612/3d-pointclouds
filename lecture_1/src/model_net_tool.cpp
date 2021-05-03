//
// Created by Zhang Zhimeng on 2021/3/6.
//
#include "model_net_tool.h"

#include <iostream>
#include <fstream>

bool ModelNetTool::ReadData(const std::string &file_path, ModelData& model_data) {
    std::fstream model_file(file_path, std::ios::in);

    if (!model_file.is_open()){
        std::cout << "failure to open model file: " << file_path << std::endl;
        return false;
    }

    model_data.vertices_.clear();
    model_data.faces_.clear();

    std::stringstream ss;
    std::string line_data;

    std::getline(model_file, line_data);
    std::getline(model_file, line_data);

    ss << line_data;

    ss >> model_data.vertex_number_;
    ss >> model_data.face_number_;
    ss >> model_data.edge_number_;
    ss.clear();

    model_data.vertices_.resize(model_data.vertex_number_);
    model_data.faces_.resize(model_data.face_number_);

    for (unsigned int i = 0; i < model_data.vertex_number_; ++i) {
        std::getline(model_file, line_data);
        ss << line_data;
        ss >> model_data.vertices_.at(i).x();
        ss >> model_data.vertices_.at(i).y();
        ss >> model_data.vertices_.at(i).z();
        ss.clear();
    }

    for (unsigned int i = 0; i < model_data.face_number_; ++i) {
        std::getline(model_file, line_data);
        ss << line_data;
        ss >> model_data.faces_.at(i).x();
        ss >> model_data.faces_.at(i).y();
        ss >> model_data.faces_.at(i).z();
        ss >> model_data.faces_.at(i).w();
        ss.clear();
    }

    return true;
}