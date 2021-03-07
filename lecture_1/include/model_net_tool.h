//
// Created by Zhang Zhimeng on 2021/3/6.
//

#ifndef LECTURE_1_MODEL_NET_TOOL_H
#define LECTURE_1_MODEL_NET_TOOL_H

#include "model_data.h"

class ModelNetTool{
public:
    ModelNetTool() = default;

    bool ReadData(const std::string& file_path, ModelData& model_data);
};

#endif //LECTURE_1_MODEL_NET_TOOL_H
