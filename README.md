# Bayesian Filtering Library

This project provides a robust implementation of various Bayesian filtering techniques, including Linear Kalman Filter, Extended Kalman Filter, Unscented Kalman Filter, and Particle Filter. It's designed for personal use for robotics applications.  
## Features

- **Linear Kalman Filter**: Implements the basic Kalman filter for linear models.
- **Extended Kalman Filter**: Supports non-linear models by linearizing about the current estimate.
- **Unscented Kalman Filter**: Uses the unscented transform to handle highly non-linear models better than EKF.
- **Particle Filter**: Implements a sequential Monte Carlo method for non-linear and non-Gaussian problems.
- **Data Generation**: Tools to generate synthetic data for testing and validation.
- **Visualization**: Integrates with modern visualization tools to provide real-time insights into filter performance.

## Getting Started

### Prerequisites

- C++ Compiler (g++ 9.4.0)
- CMake (version 3.10 or higher)
- Eigen3 Library
- Matplotlib C++

### Building the Project

To build the project, follow these steps:
```bash
mkdir build
cd build
cmake ..
make
```

This will compile all targets defined in the project including the library, examples, and tests.

### Running Examples

After building, you can run the examples:
```bash
./build/examples/demo
```
## Usage

Here is a simple example on how to use the Linear Kalman Filter:
```cpp
#include <Eigen/Dense>
#include "kf/LinearKalmanFilter.hpp"

int main() {
    Eigen::MatrixXd transition(2, 2);
    transition << 1, 1, 0, 1;
    Eigen::MatrixXd observation(1, 2);
    observation << 1, 0;
    Eigen::MatrixXd processNoise(2, 2);
    processNoise << 0.1, 0, 0, 0.1;
    Eigen::MatrixXd measurementNoise(1, 1);
    measurementNoise << 0.1;
    LinearKalmanFilter kf(transition, observation, processNoise, measurementNoise);
    
    Eigen::VectorXd observation(1);
    observation << 1; // Example observation data

    kf.predict();
    kf.update(observation);

    return 0;
}
```
## Contributing

Contributions to enhance the functionality or efficiency of this filtering library are welcome. Please feel free to fork the repository, make changes, and submit a pull request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

- **Yunus Talha Erzurumlu** - *Primary Developer* - [yunusstalha](https://github.com/yunusstalha)

## Acknowledgments
- Thanks Dr. Umut Orguner and Dr. Emre Ozkan, for their lectures notes to make this topic clear for me.

## References

- For a detailed introduction to Kalman filters and Bayesian filtering techniques, see the EE793 lecture notes by Umut Orguner available [here](https://users.metu.edu.tr/umut/ee793/files/METULecture1.pdf).

