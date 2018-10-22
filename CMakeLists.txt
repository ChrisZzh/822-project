cmake_minimum_required(VERSION 3.12)
project(16822_project)

set(CMAKE_CXX_STANDARD 14)

include_directories(../Eigen)

add_executable(822-project main.cpp LidarPlaneExtractor.cpp LidarPlaneExtractor.h DataStructures.h)