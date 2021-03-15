//
// Created by Zhang Zhimeng on 2021/3/15.
//

#ifndef LECTURE_2_KNN_RESULT_SET_H
#define LECTURE_2_KNN_RESULT_SET_H

#include <vector>
#include <limits>

class DistIndex {
public:
    DistIndex(float distance, unsigned int index)
            : distance_(distance), index_(index) {}

    bool LessThan(float other) {
        return distance_ < other;
    }

public:
    float distance_;
    unsigned int index_;
};

class KNNResultSet {
public:
    KNNResultSet(unsigned int capacity);

    unsigned int Size();

    bool Full();

    float WorstDist();

    void AddPoint(const float dist, const unsigned int index);

private:
    unsigned int capacity_;
    unsigned int count_ = 0;
    float worst_dist_ = std::numeric_limits<float>::max();
    std::vector<DistIndex> dist_index_list_;
};

#endif //LECTURE_2_KNN_RESULT_SET_H
