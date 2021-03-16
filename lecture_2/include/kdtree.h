//
// Created by Zhang Zhimeng on 2021/3/13.
//

#ifndef LECTURE_2_KDTREE_H
#define LECTURE_2_KDTREE_H

#include "knn_result_set.h"
#include "data_type.h"
#include <vector>
#include <limits>

class KDTree{
public:
    KDTree();

    void BuildTree(const PointCloudPtr& point_cloud_ptr, unsigned int leaf_size);

    std::vector<unsigned int> QueryNearestNeighbor(const int number, const Eigen::Vector3f& query_point);


private:
    enum AXIS{X=0, Y, Z};

    struct Node{
        Node(AXIS axis, double value, Node*& left, Node*& right,
             const std::vector<unsigned int>& point_indices)
             : axis_(axis), value_(value), left_ptr_(left)
             , right_ptr_(right), point_indices_(point_indices){}

        AXIS axis_;
        float value_ = std::numeric_limits<float>::max();
        Node* left_ptr_ = nullptr;
        Node* right_ptr_ = nullptr;
        std::vector<unsigned int> point_indices_;

        bool IsLeaf() const{
            return value_ == std::numeric_limits<float>::min();
        }
    };

    Node* node_ptr_ = nullptr;

    Node* KDTreeRecursiveBuild(Node*& root, const PointCloudPtr& point_cloud_ptr,
                               const std::vector<unsigned int>& point_indices,
                               AXIS axis, unsigned int leaf_size);

    std::vector<unsigned int> SortKeyByValue(const std::vector<unsigned int>& point_indices,
                                             const PointCloudPtr& point_cloud_ptr,
                                             const AXIS& axis);

    AXIS AxisRoundRobin(const AXIS axis){
        constexpr unsigned int dim = 3;
        AXIS axis_temp;
        if (axis == (AXIS)dim-1){
            axis_temp = AXIS::X;
        } else{
            axis_temp =(AXIS)((int)axis + 1);
        }

        return axis_temp;
    }

    bool KNNSearch(Node*& root, const PointCloudPtr& point_cloud_ptr,
                          KNNResultSet& knn_result_set,
                          const Eigen::Vector3f& query_point);

private:
    PointCloudPtr point_cloud_ptr_;
};

#endif //LECTURE_2_KDTREE_H
