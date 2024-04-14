#pragma once
#include <Eigen/Dense>

class BaseFilter {
public:
    virtual ~BaseFilter() {}

    // Initialize filter state
    virtual void initialize(const Eigen::VectorXd& initialState, const Eigen::MatrixXd& initialCovariance) = 0;

    // Predict the state and covariance
    virtual void predict() = 0;

    // Correct the state and covariance based on the observation
    virtual void update(const Eigen::VectorXd& observation) = 0;
};
