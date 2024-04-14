#include <iostream>
#include <Eigen/Dense>
#include "kf/LinearKalmanFilter.hpp"


int main(){
    // Create a new Kalman filter
    Eigen::MatrixXd transition(2, 2);
    transition << 1, 1, 0, 1;
    Eigen::MatrixXd observation(1, 2);
    observation << 1, 0;
    Eigen::MatrixXd processNoise(2, 2);
    processNoise << 0.1, 0, 0, 0.1;
    Eigen::MatrixXd measurementNoise(1, 1);
    measurementNoise << 0.1;
    LinearKalmanFilter kf(transition, observation, processNoise, measurementNoise);
    
    return 0;
}