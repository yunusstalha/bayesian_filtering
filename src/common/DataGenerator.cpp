#include "common/DataGenerator.hpp"


// Function to create a constant velocity model
DynamicModel CreateConstantVelocityModel(double dt) {
    DynamicModel model;
    model.transitionMatrix = Eigen::MatrixXd::Identity(4, 4);
    model.transitionMatrix(0, 1) = dt;
    model.transitionMatrix(2, 3) = dt;

    model.controlInputMatrix = Eigen::MatrixXd::Identity(4, 2);
    model.controlInputMatrix(0, 0) = dt * dt / 2;
    model.controlInputMatrix(1, 0) = dt;
    model.controlInputMatrix(2, 1) = dt * dt / 2;
    model.controlInputMatrix(3, 1) = dt;

    model.processNoiseCovariance = Eigen::MatrixXd::Identity(2, 2);
    model.processNoiseCovariance *= 0.1;  // Adjust this value based on expected process noise level

    model.measurementMatrix = Eigen::MatrixXd::Identity(4, 4);

    model.measurementNoiseCovariance = Eigen::MatrixXd::Identity(4, 4);
    model.measurementNoiseCovariance *= 0.1;  // Adjust this value based on expected measurement noise level

    return model;
}


DataGenerator::DataGenerator(const Eigen::VectorXd& initialState)
    : state_(initialState), distribution_(0.0, 1.0) {
    std::random_device rd;
    generator_.seed(rd());
    model_ = CreateConstantVelocityModel(1.0);  // Default model is constant velocity
}

Eigen::VectorXd DataGenerator::generateMeasurement() {
    // Measurement noise generation
    Eigen::VectorXd measurementNoise(model_.measurementMatrix.rows());
    for (int i = 0; i < measurementNoise.size(); ++i) {
        measurementNoise(i) = std::sqrt(model_.measurementNoiseCovariance(i, i)) * distribution_(generator_);
    }
    // y = Cx + V
    return model_.measurementMatrix * state_ + measurementNoise;
}

Eigen::VectorXd DataGenerator::step() {
    // Process noise generation
    Eigen::VectorXd processNoise(model_.controlInputMatrix.cols());
    for (int i = 0; i < processNoise.size(); ++i) {
        processNoise(i) = std::sqrt(model_.processNoiseCovariance(i, i)) * distribution_(generator_);
    }
    // x = Ax + Bu + W (with u typically being 0 in this model)
    state_ = model_.transitionMatrix * state_ + model_.controlInputMatrix * processNoise;
    return state_;
}



// [TODO] : Implement similar functions for constant position and constant acceleration models
