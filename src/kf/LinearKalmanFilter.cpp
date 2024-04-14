#include "kf/LinearKalmanFilter.hpp"

LinearKalmanFilter::LinearKalmanFilter(const Eigen::MatrixXd& transition, const Eigen::MatrixXd& observation,
                                       const Eigen::MatrixXd& processNoise, const Eigen::MatrixXd& measurementNoise)
    : transitionMatrix_(transition), observationMatrix_(observation),
      processNoise_(processNoise), measurementNoise_(measurementNoise) {}

void LinearKalmanFilter::initialize(const Eigen::VectorXd& initialState, const Eigen::MatrixXd& initialCovariance) {
    state_ = initialState;
    covariance_ = initialCovariance;
}


void LinearKalmanFilter::predict() {
// Performs the prediction step of the Kalman filter, updating the state and covariance
// based on the model prediction alone, without incorporating new measurements.

    state_ = transitionMatrix_ * state_; // Predicted state estimate. x = A * x 

    covariance_ = transitionMatrix_ * covariance_ * transitionMatrix_.transpose() + processNoise_; // Predicted covariance estimate. P = A * P * A^T + W
}



void LinearKalmanFilter::update(const Eigen::VectorXd& observation) {
// Incorporates a new observation into the state estimate, updating both the state
// and the covariance to reflect the new information.

    // Compute the Kalman gain K = P * C^T * (C * P * C^T + V)^-1
    Eigen::MatrixXd S = observationMatrix_ * covariance_ * observationMatrix_.transpose() + measurementNoise_;
    Eigen::MatrixXd K = covariance_ * observationMatrix_.transpose() * S.inverse();

    // Update the state estimate.  x = x + K * (y - C * x)
    state_ = state_ + K * (observation - observationMatrix_ * state_);

    // Update the covariance estimate. P = (I - K * C) * P
    covariance_ = (Eigen::MatrixXd::Identity(state_.size(), state_.size()) - K * observationMatrix_) * covariance_;
}
