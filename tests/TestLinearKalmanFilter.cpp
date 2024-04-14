#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "kf/LinearKalmanFilter.hpp"

class LinearKalmanFilterTest : public ::testing::Test {
protected:
LinearKalmanFilterTest() {};    // Create a new Kalman filter instance before each test case
    void SetUp() override {
        transition_ = Eigen::MatrixXd(2, 2);
        transition_ << 1, 1, 0, 1;

        observation_ = Eigen::MatrixXd(1, 2);
        observation_ << 1, 0;

        processNoise_ = Eigen::MatrixXd(2, 2);
        processNoise_ << 0.1, 0, 0, 0.1;

        measurementNoise_ = Eigen::MatrixXd(1, 1);
        measurementNoise_ << 0.1;

        initialState_ = Eigen::VectorXd(2);
        initialState_ << 0, 0;

        initialCovariance_ = Eigen::MatrixXd(2, 2);
        initialCovariance_ << 1, 0, 0, 1;

        // Construct LinearKalmanFilter object with parameters
        kf_ = LinearKalmanFilter(transition_, observation_, processNoise_, measurementNoise_);
        kf_.initialize(initialState_, initialCovariance_);
    }

    // Member variables accessible to all test cases
    Eigen::MatrixXd transition_;
    Eigen::MatrixXd observation_;
    Eigen::MatrixXd processNoise_;
    Eigen::MatrixXd measurementNoise_;
    Eigen::VectorXd initialState_;
    Eigen::MatrixXd initialCovariance_;
    LinearKalmanFilter kf_;
};
// Test case to verify the predict function of LinearKalmanFilter
TEST_F(LinearKalmanFilterTest, PredictTest) {
    kf_.predict();

    // [TODO] : Add assertions to verify the correctness of the predicted state and covariance
}

// Test case to verify the update function of LinearKalmanFilter
TEST_F(LinearKalmanFilterTest, UpdateTest) {
    Eigen::VectorXd observation(1);
    observation << 1; // Example observation data

    kf_.predict();
    kf_.update(observation);

    // [TODO] : Add assertions to verify the correctness of the updated state and covariance
}

// Main function to run all the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
