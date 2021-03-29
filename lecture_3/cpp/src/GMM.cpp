//
// Created by Zhang Zhimeng on 2021/3/28.
//
#include "GMM.h"
#include <random>

GMM::GMM(unsigned int K, unsigned int max_iteration, double threshold)
        : K_(K), max_iterations_(max_iteration), threshold_(threshold) {}

void GMM::InitPara() {
    clusters_.resize(K_);
    unsigned int number_points = source_points_.size();
    W_.resize(number_points, K_);

    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_int_distribution<std::mt19937::result_type> distribution(0, number_points - 1);

    for (unsigned int i = 0; i < K_; ++i) {
        Eigen::VectorXd center = source_points_[distribution(rng)];
        clusters_[i].mu_ = center;

        Eigen::VectorXd cov_vec;
        cov_vec.resize(dimension_);
        cov_vec.setOnes();

        clusters_[i].covariance_ = cov_vec.asDiagonal();
        clusters_[i].pi_ = 1.0 / K_;
    }
}

void GMM::Fit(const std::vector<Eigen::VectorXd> &source_points) {
    source_points_ = source_points;
    dimension_ = source_points_[0].rows();

    InitPara();

    double last_log_likelihood = 0.0;
    for (unsigned int i = 0; i < max_iterations_; ++i) {
        double curr_log_likelihood = UpdateW();
        double delta_log_likelihood = curr_log_likelihood - last_log_likelihood;
        last_log_likelihood = curr_log_likelihood;

        std::cout << delta_log_likelihood << std::endl;
        if (std::abs(delta_log_likelihood) < threshold_) {
            std::cout << "End of iteration, a total of iterations: " << i << std::endl;
            break;
        }

        UpdatePi();
        UpdateMu();
        UpdateVar();
    }
}

std::vector<GMM::Cluster> GMM::GetClusters() {
    for (unsigned int i = 0; i < W_.rows(); ++i) {
        double max_value = std::numeric_limits<double>::min();
        int index = -1;
        for (unsigned int j = 0; j < K_; ++j) {
            if (W_(i, j) > max_value) {
                max_value = W_(i, j);
                index = j;
            }
        }

        clusters_[index].points_.emplace_back(source_points_[i]);
    }

    return clusters_;
}

double GMM::PDF(const Eigen::VectorXd &x,
                const Eigen::VectorXd &mean,
                const Eigen::MatrixXd &covariance) {
    double part_1 = 1.0 / std::pow(2 * M_PI, x.rows() / 2.0);
    double part_2 = 1.0 / covariance.squaredNorm();
    double part_3 = std::exp(-0.5 * (x - mean).transpose() * covariance.inverse() * (x - mean));
    return part_1 * part_2 * part_3;
}

double GMM::EvalMultivNorm(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec, const Eigen::MatrixXd &covMat) {
    const double logSqrt2Pi = 0.5 * std::log(2 * M_PI);
    typedef Eigen::LLT<Eigen::MatrixXd> Chol;
    Chol chol(covMat);

    if (chol.info() != Eigen::Success) throw "decomposition failed!";
    const Chol::Traits::MatrixL &L = chol.matrixL();
    double quadform = (L.solve(x - meanVec)).squaredNorm();
    return std::exp(-x.rows() * logSqrt2Pi - 0.5 * quadform) / L.determinant();
}

void GMM::UpdateMu() {
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        for (unsigned int j = 0; j < K_; ++j) {
            clusters_[j].mu_ += W_(i, j) * source_points_[i];
        }
    }

    for (unsigned int i = 0; i < K_; ++i) {
        clusters_[i].mu_ /= (source_points_.size() * clusters_[i].pi_);
    }
}

void GMM::UpdatePi() {
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        for (unsigned int j = 0; j < K_; ++j) {
            clusters_[j].pi_ += W_(i, j);
        }
    }

    for (unsigned int i = 0; i < K_; ++i) {
        clusters_[i].pi_ /= static_cast<double>(source_points_.size());
    }
}

void GMM::UpdateVar() {
    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        for (unsigned int j = 0; j < K_; ++j) {
            clusters_[j].covariance_ += W_(i, j) * (source_points_[i] - clusters_[j].mu_) *
                                        (source_points_[i] - clusters_[j].mu_).transpose();
        }
    }

    for (unsigned int i = 0; i < K_; ++i) {
        clusters_[i].covariance_ /= (source_points_.size() * clusters_[i].pi_);
    }
}

///TODO: 该函数小概率情况下会导致概率值为无穷大,后续可以针对该函数进行优化,可能是PDF()函数的实现有一些问题
///TODO: 另外对于GMM的奇异值问题,该算法的实现中并没有考虑进去
double GMM::UpdateW() {
    double log_likelihood = 0.0;

    for (unsigned int i = 0; i < source_points_.size(); ++i) {
        double sum = 0.0;
        for (unsigned int j = 0; j < K_; ++j) {
//            W_(i, j) = clusters_[j].pi_ * PDF(source_points_[i], clusters_[j].mu_, clusters_[j].covariance_);// My pdf
            W_(i, j) = clusters_[j].pi_ * EvalMultivNorm(source_points_[i], clusters_[j].mu_, clusters_[j].covariance_);
            sum += W_(i, j);
        }

        for (unsigned int j = 0; j < K_; ++j) {
            W_(i, j) /= sum;
        }

        log_likelihood += std::log(sum);
    }

    return log_likelihood;
}