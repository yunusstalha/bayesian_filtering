# In your tests directory's CMakeLists.txt

# Locate Google Test
find_package(GTest REQUIRED)

# Add the test executable
add_executable(test_kf TestLinearKalmanFilter.cpp)

# Link the test executable with Google Test and your library
target_link_libraries(test_kf GTest::GTest GTest::Main bayesian_filters) 

# Include directories for Eigen and your project headers
target_include_directories(test_kf 
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include # Path to your headers
                           PRIVATE ${EIGEN3_INCLUDE_DIR})              # Include Eigen 
