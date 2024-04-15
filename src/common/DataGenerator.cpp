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

#include "common/DataGenerator.hpp"
#include <iostream>

// Function to create a constant velocity model
DynamicModel CreateConstantVelocityModel(Eigen::MatrixXd processNoiseCovariance, 
                                        Eigen::MatrixXd measurementNoiseCovariance,
                                        double dt) {
    DynamicModel model;
    Eigen::MatrixXd measurementMatrix(2,4);
    measurementMatrix << 1, 0, 0, 0,
                        0, 0, 1, 0;

    model.measurementMatrix = measurementMatrix;
    model.transitionMatrix = Eigen::MatrixXd::Identity(4, 4);
    model.transitionMatrix(0, 1) = dt;
    model.transitionMatrix(2, 3) = dt;

    model.controlInputMatrix = Eigen::MatrixXd::Identity(4, 2);
    model.controlInputMatrix(0, 0) = dt * dt / 2;
    model.controlInputMatrix(1, 0) = dt;
    model.controlInputMatrix(2, 1) = dt * dt / 2;
    model.controlInputMatrix(3, 1) = dt;

    if (processNoiseCovariance.size() > 0) {
        model.processNoiseCovariance = processNoiseCovariance;
    } else {
        model.processNoiseCovariance = Eigen::MatrixXd::Identity(4, 4);
        model.processNoiseCovariance *= 0.1;  // Adjust this value based on expected process noise level
    }
    if (measurementNoiseCovariance.size() > 0) {
        model.measurementNoiseCovariance = measurementNoiseCovariance;
    } else {
        model.measurementNoiseCovariance = Eigen::MatrixXd::Identity(2, 2);
        model.measurementNoiseCovariance *= 0.1;  // Adjust this value based on expected measurement noise level
    }

    return model;
}


DataGenerator::DataGenerator(const DynamicModel model, Eigen::VectorXd& initialState)
    :model_(model), state_(initialState), distribution_(0.0, 1.0) {
    std::random_device rd;
    generator_.seed(rd());
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
    Eigen::VectorXd processNoise(model_.transitionMatrix.rows());
    // x = Ax + W (with u typically being 0 in this model)
    for (int i = 0; i < processNoise.size(); ++i) {
        processNoise(i) = std::sqrt(model_.processNoiseCovariance(i, i)) * distribution_(generator_);
    }
    state_ = model_.transitionMatrix * state_ + processNoise;
    return state_;
}

// [TODO] : Implement similar functions for constant position and constant acceleration models
