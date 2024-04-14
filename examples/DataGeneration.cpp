#include "common/DataGenerator.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <vector>

namespace plt = matplotlibcpp;

int main() {
    // Initialize the data generator as before
    double dt = 1.0;
    Eigen::VectorXd initialState(4);
    initialState << 0, 1, 0, 1;  // Starting at origin, moving with velocity of 1 in both x and y

    // Initialize the data generator with the model and initial state
    DataGenerator dataGenerator(initialState);

    // Prepare vectors to store the data for plotting
    std::vector<double> state_x, state_y, measurements_x, measurements_y;

    // Number of steps to simulate
    int steps = 50;
    for (int i = 0; i < steps; ++i) {
        Eigen::VectorXd newState = dataGenerator.step();
        Eigen::VectorXd newMeasurement = dataGenerator.generateMeasurement();

        // Store data
        state_x.push_back(newState(0));  // x position of state
        state_y.push_back(newState(2));  // y position of state
        measurements_x.push_back(newMeasurement(0));  // x measurement
        measurements_y.push_back(newMeasurement(2));  // y measurement
    }

    // Plot positions in a 2D plane
    plt::figure_size(1200, 780);
    plt::named_plot("State Trajectory", state_x, state_y, "b-");
    plt::named_plot("Measurement Trajectory", measurements_x, measurements_y, "r.");
    plt::xlabel("X Position");
    plt::ylabel("Y Position");
    plt::title("2D Trajectories: State vs. Measurement");
    plt::legend();
    plt::show();

    return 0;
}


