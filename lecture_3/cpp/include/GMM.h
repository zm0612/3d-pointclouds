//
// Created by Zhang Zhimeng on 2021/3/28.
//

#ifndef CLUSTER_GMM_H
#define CLUSTER_GMM_H

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

class GMM {
public:

    struct Cluster{
        Eigen::VectorXd mu_;
        Eigen::MatrixXd covariance_;
        double pi_;
    };

    GMM(unsigned int K, unsigned int max_iteration);

    void Fit(const std::vector<Eigen::VectorXd> &source_points);

private:
    void InitPara();

    double UpdateW();

    void UpdatePi();

    void UpdateMu();

    void UpdateVar();

    double PDF(const Eigen::VectorXd& x,
               const Eigen::VectorXd& mean,
               const Eigen::MatrixXd& covariance);

    double evalMultivNorm(const Eigen::VectorXd &x,
                          const Eigen::VectorXd &meanVec,
                          const Eigen::MatrixXd &covMat);

private:
    unsigned int K_;
    unsigned int dimension_;
    unsigned int max_iterations_;
    std::vector<Cluster> clusters_;
    std::vector<Eigen::VectorXd> source_points_;
    Eigen::MatrixXd W_;
};

#endif //CLUSTER_GMM_H
