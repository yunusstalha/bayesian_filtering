# Gather all source files in the src directory and its subdirectories
file(GLOB_RECURSE SOURCES "*.cpp")

add_library(bayesian_filters
    kf/LinearKalmanFilter.cpp
    # Add other source files here
)

# Make sure the library can find its headers and Eigen
target_include_directories(bayesian_filters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(bayesian_filters PUBLIC Eigen3::Eigen)


