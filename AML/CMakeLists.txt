cmake_minimum_required(VERSION 3.14)

# Set your project name here
project( AML )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wunused -std=c++11")

set(SRC_CPP_AML
    AML.h 
    vector3.h
    matrix3.h
    dcm.h 
    euler.h
    AML.cpp
    vector3.cpp
    matrix3.cpp
    dcm.cpp
    euler.cpp
    quaternion.h
    quaternion.cpp
)

add_library(
    ${PROJECT_NAME} STATIC 
    ${SRC_CPP_AML}
)

target_include_directories(
    ${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR}
)