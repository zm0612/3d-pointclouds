//
// Created by Zhang Zhimeng on 2021/3/13.
//
#include "kdtree.h"

#include <algorithm>

bool Compare(std::pair<unsigned int, float> a, std::pair<unsigned int, float> b) {
    return a.second < b.second;
}

KDTree::KDTree():point_cloud_ptr_(new PointCloud) {}

void KDTree::BuildTree(const PointCloudPtr &point_cloud_ptr, unsigned int leaf_size) {
    point_cloud_ptr_ = point_cloud_ptr;

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
        Node* left_node = nullptr, *right_node = nullptr;
        Node *node = new Node(axis, std::numeric_limits<float>::min(), left_node, right_node, point_indices);
        root = node;
    }

    if (point_indices.size() > leaf_size) {
        std::vector<unsigned int> sorted_indices = SortKeyByValue(point_indices, point_cloud_ptr, axis);

        unsigned int middle_left_idx = std::ceil(sorted_indices.size() / 2.0) - 1u;
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

        std::vector<unsigned int> sorted_indices_left(sorted_indices.begin(), sorted_indices.begin() + middle_right_idx);
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
                                                           point_cloud_ptr->at(point_indices[i]).y);
                key_values[i] = (key_value_y);
            }
                break;
            case AXIS::Z: {
                std::pair<unsigned int, float> key_value_z(point_indices[i],
                                                           point_cloud_ptr->at(point_indices[i]).z);
                key_values[i] = (key_value_z);
            }
                break;
        }
    }

    std::stable_sort(key_values.begin(), key_values.end(), Compare);

    std::vector<unsigned int> sorted_indices(point_indices.size());
    for (unsigned int i = 0; i < key_values.size(); ++i) {
        sorted_indices[i] = key_values[i].first;
    }

    return sorted_indices;
}

bool KDTree::KNNSearch(Node *&root, const PointCloudPtr &point_cloud_ptr,
                   KNNResultSet &knn_result_set, const Eigen::Vector3f &query_point) {
    if (root == nullptr){
        return false;
    }

    if (root->IsLeaf()){
        for (unsigned int i = 0; i < root->point_indices_.size(); ++i) {
            Eigen::Vector3f point;
            point.x() = point_cloud_ptr->at(root->point_indices_[i]).x;
            point.y() = point_cloud_ptr->at(root->point_indices_[i]).y;
            point.z() = point_cloud_ptr->at(root->point_indices_[i]).z;
            float diff = (query_point - point).norm();
            knn_result_set.AddPoint(diff, root->point_indices_[i]);
        }
        return false;
    }

    float query_point_axis;
    switch (root->axis_) {
        case AXIS::X:
            query_point_axis = query_point.x();
            break;
        case AXIS::Y:
            query_point_axis = query_point.y();
            break;
        case AXIS::Z:
            query_point_axis = query_point.z();
            break;
    }

    if (query_point_axis <= root->value_){
        KNNSearch(root->left_ptr_, point_cloud_ptr, knn_result_set, query_point);
        if (std::abs(query_point_axis - root->value_) < knn_result_set.WorstDist()){
            KNNSearch(root->right_ptr_, point_cloud_ptr, knn_result_set, query_point);
        }
    } else{
        KNNSearch(root->right_ptr_, point_cloud_ptr, knn_result_set, query_point);
        if (std::abs(query_point_axis - root->value_) < knn_result_set.WorstDist()){
            KNNSearch(root->left_ptr_, point_cloud_ptr, knn_result_set, query_point);
        }
    }

    return false;
}

std::vector<unsigned int> KDTree::QueryNearestNeighbor(const int number, const Eigen::Vector3f &query_point) {
    KNNResultSet knn_result_set(number);

    KNNSearch(node_ptr_, point_cloud_ptr_, knn_result_set, query_point);

    for (int i = 0; i < knn_result_set.dist_index_list_.size(); ++i) {
        std::cout << "min dist: " << knn_result_set.dist_index_list_[i].distance_ << std::endl;
    }

    return knn_result_set.GetPointIndices();
}