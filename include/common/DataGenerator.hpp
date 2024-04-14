#pragma once

#include <Eigen/Dense>
#include <random>

// Structure to hold model parameters
struct DynamicModel {
    Eigen::MatrixXd transitionMatrix;  // State transition matrix (A)
    Eigen::MatrixXd controlInputMatrix; // Control input matrix (B, not used in this model)
    Eigen::MatrixXd processNoiseCovariance;  // Process noise covariance matrix (W)
    Eigen::MatrixXd measurementMatrix;  // Measurement matrix (C)
    Eigen::MatrixXd measurementNoiseCovariance;  // Measurement noise covariance matrix (V)
};

// Class to generate data based on a dynamic model
class DataGenerator {
public:
    DataGenerator(const Eigen::VectorXd& initialState);

    // Generates a noisy measurement based on the current state (y = Cx + V)
    Eigen::VectorXd generateMeasurement();

    // Advances the state based on the dynamic model (x = Ax + Bu + W, with u not used)
    Eigen::VectorXd step();

private:
    DynamicModel model_;
    Eigen::VectorXd state_; // State vector (x)
    std::default_random_engine generator_;
    std::normal_distribution<double> distribution_;
};