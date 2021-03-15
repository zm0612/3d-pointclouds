//
// Created by Zhang Zhimeng on 2021/3/15.
//
#include "knn_result_set.h"

KNNResultSet::KNNResultSet(unsigned int capacity)
         :capacity_(capacity){
    for (unsigned int i = 0; i < capacity_; ++i) {
        dist_index_list_.emplace_back(DistIndex(worst_dist_, 0));
    }
}

bool KNNResultSet::Full() {
    return count_ == capacity_;
}

float KNNResultSet::WorstDist() {
    return worst_dist_;
}

unsigned int KNNResultSet::Size() {
    return count_;
}

void KNNResultSet::AddPoint(const float dist, const unsigned int index) {
    if (dist > worst_dist_){
        return;
    }

    if (count_ < capacity_){
        ++count_;
    }

    unsigned int i = count_ - 1;
    while (i > 0){
        if (dist_index_list_[i-1].distance_ > dist){
            dist_index_list_[i] = dist_index_list_[i-1];
            --i;
        } else{
            break;
        }
    }

    dist_index_list_[i].distance_ = dist;
    dist_index_list_[i].index_ = index;
    worst_dist_ = dist_index_list_[capacity_-1].distance_;
}
