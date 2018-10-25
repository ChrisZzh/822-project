cmake_minimum_required(VERSION 3.12)
project(16822_project)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.3 REQUIRED COMPONENTS)

include_directories(../Eigen ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(822-project main.cpp LidarPlaneExtractor.cpp LidarPlaneExtractor.h DataStructures.h)

target_link_libraries(822-project ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
