#pragma once
#include "common/BaseFilter.hpp"

/**
 * Implements a Linear Kalman Filter using the state-space representation.
 * The model equations are:
 *   State Update Equation: x = Ax + Bu + W
 *   Measurement Equation: y = Cx + V
 * Where:
 *   x: State vector
 *   A: State transition matrix
 *   B: Control input matrix (not used in this implementation)
 *   u: Control vector (not used in this implementation)
 *   W: Process noise vector
 *   y: Measurement vector
 *   C: Observation matrix
 *   V: Measurement noise vector
 *   K: Kalman gain matrix
 *   P: State covariance matrix
 *
 * This class provides methods to initialize the filter, predict the next state,
 * and update the state based on new measurements.
 */


class LinearKalmanFilter : public BaseFilter {

public:
    LinearKalmanFilter() = default;
    // Constructor initializes the filter with the necessary system matrices.
    explicit LinearKalmanFilter(const Eigen::MatrixXd& transition, const Eigen::MatrixXd& observation,
                                const Eigen::MatrixXd& processNoise, const Eigen::MatrixXd& measurementNoise);

    // Initializes the filter state and covariance matrices.
    void initialize(const Eigen::VectorXd& initialState, const Eigen::MatrixXd& initialCovariance) override;

    // Predicts the next state and covariance based on the current state and system model.
    void predict() override;

    // Updates the state by incorporating a new observation, adjusting the state and covariance.
    void update(const Eigen::VectorXd& observation) override;

    // Accessor methods to get the current state and covariance.
    Eigen::VectorXd getState() const  { return state_; }
    Eigen::MatrixXd getCovariance() const override { return covariance_; }


private:
    Eigen::VectorXd state_;            // State vector (x) of the system 
    Eigen::MatrixXd covariance_;       // Covariance matrix (P) of the state
    Eigen::MatrixXd processNoise_;     // Process noise covariance matrix (W)
    Eigen::MatrixXd measurementNoise_; // Measurement noise covariance matrix (V)
    Eigen::MatrixXd transitionMatrix_; // State transition matrix (A)
    Eigen::MatrixXd observationMatrix_; // Observation matrix (C)
};