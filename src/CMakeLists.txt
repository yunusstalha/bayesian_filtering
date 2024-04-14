# Gather all source files in the src directory and its subdirectories
file(GLOB_RECURSE SOURCES "*.cpp")

# Define the library that uses these sources
add_library(bayesian_filters ${SOURCES})

# Make sure the library can find its headers and Eigen
target_include_directories(bayesian_filters PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(bayesian_filters PUBLIC Eigen3::Eigen)


