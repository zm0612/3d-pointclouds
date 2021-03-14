//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "kdtree.h"

#include <algorithm>

bool Compare(std::pair<unsigned int, float> a, std::pair<unsigned int, float> b) {
    return a.second < b.second;
}

void KDTree::BuildTree(const PointCloudPtr &point_cloud_ptr, unsigned int leaf_size) {

}

KDTree::Node *KDTree::KDTreeRecursiveBuild(Node *root, const PointCloudPtr &point_cloud_ptr,
                                           const std::vector<unsigned int> &point_indices, AXIS axis,
                                           unsigned int leaf_size) {
    if (root == nullptr) {
        Node node(axis, std::numeric_limits<float>::max(), nullptr, nullptr, point_indices);
        root = &node;
    }

    if (point_indices.size() > leaf_size) {
        auto sorted_indices = SortKeyByValue(point_indices, point_cloud_ptr, axis);

        unsigned int middle_left_idx = std::ceil(sorted_indices.size() / 2.0 - 1);
        unsigned int middle_left_point_idx = sorted_indices[middle_left_idx];
        float middle_left_point_value;

        unsigned int middle_right_idx = middle_left_idx + 1;
        unsigned int middle_right_point_idx = sorted_indices[middle_right_idx];
        float middle_right_point_value;

        switch (axis) {
            case AXIS::X:
                middle_left_point_value = point_cloud_ptr->at(middle_left_point_idx).x;
                middle_right_point_value = point_cloud_ptr->at(middle_right_point_idx).x;
                break;
            case AXIS::Y:
                middle_left_point_value = point_cloud_ptr->at(middle_left_point_idx).y;
                middle_right_point_value = point_cloud_ptr->at(middle_right_point_idx).y;
                break;
            case AXIS::Z:
                middle_left_point_value = point_cloud_ptr->at(middle_left_point_idx).z;
                middle_right_point_value = point_cloud_ptr->at(middle_right_point_idx).z;
                break;
        }

        root->value_ = (middle_right_point_value + middle_left_point_value) * 0.5f;

        ///TODO: 交替变换xyz轴
        root->left_ptr_ = KDTreeRecursiveBuild(root->left_ptr_, point_cloud_ptr,
                                               std::vector<unsigned int>(sorted_indices.begin(),
                                                                         sorted_indices.begin() + middle_left_idx),
                                               axis, leaf_size);

        root->right_ptr_ = KDTreeRecursiveBuild(root->right_ptr_, point_cloud_ptr,
                                                std::vector<unsigned int>(sorted_indices.begin() + middle_right_idx,
                                                                          sorted_indices.end()),
                                                axis, leaf_size);
    }

    return root;
}

std::vector<unsigned int> KDTree::SortKeyByValue(const std::vector<unsigned int> &point_indices,
                                                 const PointCloudPtr &point_cloud_ptr,
                                                 const AXIS &axis) {

    std::vector<std::pair<unsigned int, float>> key_values(point_indices.size());
    for (int i = 0; i < point_indices.size(); ++i) {
        switch (axis) {
            case AXIS::X: {
                std::pair<unsigned int, float> key_value_x(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values.emplace_back(key_value_x);
            }
                break;
            case AXIS::Y: {
                std::pair<unsigned int, float> key_value_y(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values.emplace_back(key_value_y);
            }
                break;
            case AXIS::Z: {
                std::pair<unsigned int, float> key_value_z(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values.emplace_back(key_value_z);
            }
                break;
        }
    }

    std::stable_sort(key_values.begin(), key_values.end(), Compare);

    std::vector<unsigned int> sorted_indices(point_indices.size());
    for (int i = 0; i < key_values.size(); ++i) {
        sorted_indices.push_back(key_values[i].first);
    }

    return sorted_indices;
}