#pragma once
#include "common/BaseFilter.h"

class LinearKalmanFilter : public BaseFilter {
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd processNoise_;
    Eigen::MatrixXd measurementNoise_;
    Eigen::MatrixXd transitionMatrix_;
    Eigen::MatrixXd observationMatrix_;

public:
    LinearKalmanFilter(const Eigen::MatrixXd& transition, const Eigen::MatrixXd& observation,
                       const Eigen::MatrixXd& processNoise, const Eigen::MatrixXd& measurementNoise);

    void initialize(const Eigen::VectorXd& initialState, const Eigen::MatrixXd& initialCovariance) override;

    void predict() override;

    void correct(const Eigen::VectorXd& observation) override;
};
