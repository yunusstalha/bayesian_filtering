cmake_minimum_required(VERSION 3.10)
project(BayesianFiltering)

set(CMAKE_CXX_STANDARD 17)

# Eigen3 library is required for this project
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include the headers
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
# Add subdirectories for each component of the project
add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(tests)
