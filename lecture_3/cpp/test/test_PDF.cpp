//
// Created by Zhang Zhimeng on 2021/3/28.
//
#include <eigen3/Eigen/Dense>
#include <iostream>

double PDF(const Eigen::VectorXd &x,
                const Eigen::VectorXd &mean,
                const Eigen::MatrixXd &covariance) {
    double part_1 = 1.0 / std::pow(2 * M_PI, x.rows() / 2.0);
    double part_2 = 1.0 / covariance.squaredNorm();
    double part_3 = std::exp(-0.5 * (x - mean).transpose() * covariance.inverse() * (x - mean));
    return part_1 * part_2 * part_3;
}

double evalMultivNorm(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec, const Eigen::MatrixXd &covMat) {
    const double logSqrt2Pi = 0.5 * std::log(2 * M_PI);
    typedef Eigen::LLT<Eigen::MatrixXd> Chol;
    Chol chol(covMat);

    if (chol.info() != Eigen::Success) throw "decomposition failed!";
    const Chol::Traits::MatrixL &L = chol.matrixL();
    double quadform = (L.solve(x - meanVec)).squaredNorm();
    return std::exp(-x.rows() * logSqrt2Pi - 0.5 * quadform) / L.determinant();
}

int main(void){
    Eigen::VectorXd x = Eigen::Vector2d::Zero();
    Eigen::VectorXd mean = Eigen::Vector2d::Zero();
    Eigen::MatrixXd cov = (Eigen::Vector2d::Ones() * 0.5).asDiagonal();

    double p_0 = PDF(x, mean, cov);
    double p_1 = evalMultivNorm(x, mean, cov);

    std::cout << p_0 << std::endl;
    std::cout << p_1 << std::endl;

    return 0;
}