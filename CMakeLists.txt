
cmake_minimum_required(VERSION 3.10)

# Set your project name here
project( AttitudeMathLib )

# Set the C++ standard to use (change to your preferred version)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})


add_subdirectory(AML)
add_subdirectory(example)
add_subdirectory(test)

# Optionally, you can enable testing with Google Test / Catch2
enable_testing()
add_test(AML_Test test/AML_Test)

# Optionally, set the output directory for the built binaries
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

