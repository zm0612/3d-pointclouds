//
// Created by Zhang Zhimeng on 2021/3/7.
//
#include "surface_normal_estimation.h"
#include "principle_component_analysis.h"

Vector3ds SurfaceNormalEstimation::CalculateNormalVector(const ModelData::TypeVertexVector &points,
                                                         const double radius_threshold) {
    Vector3ds normal_vectors;
    normal_vectors.reserve(points.size());

    for (int i = 0; i < points.size(); ++i) {
        PrincipleComponentAnalysis principle_component_analysis;
        ModelData::TypeVertexVector part_points;
        part_points.emplace_back(points[i]);
        for (int j = 0; j < points.size(); ++j) {
            if (i == j){
                continue;
            }

            double radius = (points[i] - points[j]).norm();

            if (radius <= radius_threshold){
                part_points.emplace_back(points[j]);
            }
        }

        if (part_points.size() < 3){
            normal_vectors.emplace_back(Eigen::Vector3d(0,0,0));
            continue;
        }

        principle_component_analysis.InputData(part_points);
        normal_vectors.emplace_back(principle_component_analysis.CalculateNormalVector());
    }

    return normal_vectors;
}