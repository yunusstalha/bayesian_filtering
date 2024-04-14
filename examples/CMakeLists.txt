# Add the executable
add_executable(demo main.cpp)

# Link the bayesian_filters library
target_link_libraries(demo bayesian_filters)

# Include directories for Eigen and your project headers
target_include_directories(demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
