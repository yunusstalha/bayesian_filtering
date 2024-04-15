/***************************************************************************
 * BayesianFiltering - A Modern C++ Bayesian Filtering Library
 *
 * Copyright (C) 2024 yunusstalha
 * For more information see https://github.com/yunusstalha/bayesian_filtering
 *
 * BayesianFiltering is free software: you can redistribute it and/or modify
 * it under the terms of the MIT License as published by the
 * Massachusetts Institute of Technology.
 *
 * BayesianFiltering is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * MIT License for more details.
 *
 * You should have received a copy of the MIT License
 * along with BayesianFiltering.  If not, see <https://opensource.org/licenses/MIT>.
 *
 * Author: yunusstalha 
 ***************************************************************************/

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
DynamicModel CreateConstantVelocityModel(Eigen::MatrixXd processNoiseCovariance, 
                                        Eigen::MatrixXd measurementNoiseCovariance,
                                        double dt);
class DataGenerator {
public:
    
    DataGenerator(const DynamicModel model, Eigen::VectorXd& initialState);

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