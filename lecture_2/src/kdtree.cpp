//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "kdtree.h"

#include <algorithm>

bool Compare(std::pair<unsigned int, float> a, std::pair<unsigned int, float> b) {
    return a.second < b.second;
}

void KDTree::BuildTree(const PointCloudPtr &point_cloud_ptr, unsigned int leaf_size) {
    std::vector<unsigned int> point_indices(point_cloud_ptr->size());

    for (unsigned int i = 0; i < point_cloud_ptr->size(); ++i) {
        point_indices[i] = i;
    }

    node_ptr_ = KDTreeRecursiveBuild(node_ptr_, point_cloud_ptr, point_indices, AXIS::X, leaf_size);
}

KDTree::Node *KDTree::KDTreeRecursiveBuild(Node *&root, const PointCloudPtr &point_cloud_ptr,
                                           const std::vector<unsigned int> &point_indices, AXIS axis,
                                           unsigned int leaf_size) {
    if (root == nullptr) {
        Node *node = new Node(axis, std::numeric_limits<float>::max(), nullptr, nullptr, point_indices);
        root = node;
    }

    if (point_indices.size() > leaf_size) {
        std::vector<unsigned int> sorted_indices = SortKeyByValue(point_indices, point_cloud_ptr, axis);

        unsigned int middle_left_idx = std::ceil(sorted_indices.size() / 2.0 - 1);
        unsigned int middle_left_point_idx = sorted_indices[middle_left_idx];
        float middle_left_point_value = 0.0f;

        unsigned int middle_right_idx = middle_left_idx + 1;
        unsigned int middle_right_point_idx = sorted_indices[middle_right_idx];
        float middle_right_point_value = 0.0f;

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

        std::vector<unsigned int> sorted_indices_left(sorted_indices.begin(), sorted_indices.begin() + middle_left_idx);
        root->left_ptr_ = KDTreeRecursiveBuild(root->left_ptr_, point_cloud_ptr,
                                               sorted_indices_left, AxisRoundRobin(axis),
                                               leaf_size);

        std::vector<unsigned int> sorted_indices_right(sorted_indices.begin() + middle_right_idx, sorted_indices.end());
        root->right_ptr_ = KDTreeRecursiveBuild(root->right_ptr_, point_cloud_ptr,
                                                sorted_indices_right, AxisRoundRobin(axis),
                                                leaf_size);
    }

    return root;
}

std::vector<unsigned int> KDTree::SortKeyByValue(const std::vector<unsigned int> &point_indices,
                                                 const PointCloudPtr &point_cloud_ptr,
                                                 const AXIS &axis) {

    std::vector<std::pair<unsigned int, float>> key_values(point_indices.size());
    for (unsigned int i = 0; i < point_indices.size(); ++i) {
        switch (axis) {
            case AXIS::X: {
                std::pair<unsigned int, float> key_value_x(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values[i] = (key_value_x);
            }
                break;
            case AXIS::Y: {
                std::pair<unsigned int, float> key_value_y(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values[i] = (key_value_y);
            }
                break;
            case AXIS::Z: {
                std::pair<unsigned int, float> key_value_z(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).x);
                key_values[i] = (key_value_z);
            }
                break;
        }
    }

    std::stable_sort(key_values.begin(), key_values.end(), Compare);

    std::vector<unsigned int> sorted_indices(point_indices.size());
    for (unsigned int i = 0; i < key_values.size(); ++i) {
        sorted_indices[i] = (key_values[i].first);
    }

    return sorted_indices;
}

std::vector<unsigned int> KDTree::QueryNearestNeighbor(const int number) {
    
}