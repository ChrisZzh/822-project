project(plane_extraction_tracking)
cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    std_msgs
    message_filters
    )

find_package(PCL REQUIRED)

catkin_package(
  #INCLUDE_DIRS ../include
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS PCL
  )

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime roscpp sensor_msgs stereo_msgs std_msgs)

find_package(OpenCV)
include_directories(/usr/local/include)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

link_directories(/usr/local/lib)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_executable(plane_extraction_tracking src/plane.cpp)

add_dependencies(plane_extraction_tracking ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(plane_extraction_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${libpointmatcher_LIBRARIES})
