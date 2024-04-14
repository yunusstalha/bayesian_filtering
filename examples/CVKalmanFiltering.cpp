#include "common/DataGenerator.hpp"
#include "kf/LinearKalmanFilter.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    // Initialize the data generator as before
    double dt = 0.1;
    Eigen::VectorXd initialState(4);
    initialState << 0, 1, 0, 1;  // Starting at origin, moving with velocity of 1 in both x and y
    Eigen::MatrixXd processNoiseCovariance = Eigen::MatrixXd::Identity(4, 4) * 0.001;
    Eigen::MatrixXd measurementNoiseCovariance = Eigen::MatrixXd::Identity(2, 2) * 0.02;

    auto model = CreateConstantVelocityModel(processNoiseCovariance, measurementNoiseCovariance, dt);
    // Initialize the data generator with the model and initial state
    DataGenerator dataGenerator(model, initialState);

    // Kalman filter initialization

    Eigen::MatrixXd transitionMatrix(4, 4);
    transitionMatrix << 1, dt, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, dt,
                    0, 0, 0, 1;
    Eigen::MatrixXd observationMatrix(2,4);
    observationMatrix << 1, 0, 0, 0,
                        0, 0, 1, 0;

    LinearKalmanFilter kf = LinearKalmanFilter(transitionMatrix, observationMatrix, processNoiseCovariance, measurementNoiseCovariance);
    Eigen::VectorXd initialStateKF(4);
    initialStateKF << 0, 1, 0, 1;
    Eigen::MatrixXd initialCovariance = Eigen::MatrixXd::Identity(4, 4);
    kf.initialize(initialStateKF, initialCovariance);

    // Prepare vectors to store the data for plotting
    std::vector<double> state_x, state_y, measurements_x, measurements_y;
    std::vector<double> kf_state_x, kf_state_y;
    // Number of steps to simulate
    int steps = 500;
    for (int i = 0; i < steps; ++i) {
        Eigen::VectorXd newState = dataGenerator.step();
        Eigen::VectorXd newMeasurement = dataGenerator.generateMeasurement();
        // Store data
        state_x.push_back(newState(0));  // x position of state
        state_y.push_back(newState(2));  // y position of state
        measurements_x.push_back(newMeasurement(0));  // x measurement
        measurements_y.push_back(newMeasurement(1));  // y measurement
        kf.predict();
        kf.update(newMeasurement);
        kf_state_x.push_back(kf.getState()(0));  // x position of state
        kf_state_y.push_back(kf.getState()(2));  // y position of state
    }

    

    // Plot positions in a 2D plane
    plt::figure_size(1200, 780);
    plt::named_plot("State Trajectory", state_x, state_y, "b-");
    plt::named_plot("Measurement Trajectory", measurements_x, measurements_y, "r.");
    plt::named_plot("Kalman Filtered Trajectory", kf_state_x, kf_state_y, "g-");
    plt::xlabel("X Position");
    plt::ylabel("Y Position");
    plt::title("2D Trajectories: State vs. Measurement");
    plt::legend();
    plt::grid(true);
    plt::show();

    return 0;
}


