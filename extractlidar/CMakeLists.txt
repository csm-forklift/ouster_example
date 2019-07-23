cmake_minimum_required(VERSION 2.8.3)
project(extractlidar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(lidarextract src/definezaxis.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(lidarextract ${catkin_LIBRARIES})
