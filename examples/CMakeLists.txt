cmake_minimum_required(VERSION 3.10)
project(Demo)

set(CMAKE_CXX_STANDARD 17)  # Ensure C++17 standard is used

# Assuming Eigen3 and PythonLibs have been found in the parent CMakeLists.txt
# No need to find them again if the root CMakeLists.txt already does

# Include directories for Eigen, Python, and your project headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(~/git/matplotlib-cpp)  # Adjust this path as necessary

# Add the executable
add_executable(demo DataGeneration.cpp)

# Link the bayesian_filters library if it's needed here
target_link_libraries(demo ${PYTHON_LIBRARIES} bayesian_filters)

# If matplotlib-cpp needs additional libraries, link them here
# For example, you might need to link against NumPy libraries, if used
# Find the NumPy headers
find_package(Python3 COMPONENTS NumPy)
include_directories(${Python3_NumPy_INCLUDE_DIRS})

# Depending on your project structure, if bayesian_filters library is defined elsewhere and used here,
# make sure it's linked properly, assuming it's defined in the root or another subdirectory.
