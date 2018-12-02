cmake_minimum_required(VERSION 3.12)
project(16822_project)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(../Eigen ${PCL_INCLUDE_DIRS} )

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(822-project main.cpp LidarPlaneExtractor.cpp LidarPlaneExtractor.h DataStructures.h)

target_link_libraries(822-project ${PCL_LIBRARIES} ${OpenCV_LIBS})
