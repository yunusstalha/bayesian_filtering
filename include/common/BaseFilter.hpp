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

class BaseFilter {
public:
    virtual ~BaseFilter() {}

    // Initialize filter state
    virtual void initialize(const Eigen::VectorXd& initialState, const Eigen::MatrixXd& initialCovariance) = 0;

    // Predict the state and covariance
    virtual void predict() = 0;

    // Correct the state and covariance based on the observation
    virtual void update(const Eigen::VectorXd& observation) = 0;

    // Acces Methods
    virtual Eigen::VectorXd getState() const = 0;
    virtual Eigen::MatrixXd getCovariance() const = 0;
};
