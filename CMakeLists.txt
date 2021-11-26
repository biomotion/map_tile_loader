cmake_minimum_required(VERSION 3.0.2)
project(map_tile_loader)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_loader
#  CATKIN_DEPENDS dynamic_reconfigure pcl_conversions pcl_ros roscpp rospy sensor_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include/
)

# Declare a C++ library
add_library(map_loader SHARED
  src/map_loader.cpp
)
target_include_directories(map_loader PUBLIC include)
target_link_libraries(map_loader ${catkin_LIBRARIES} jsoncpp)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node map_loader ${catkin_LIBRARIES})

add_executable(map_publisher src/map_publisher.cpp)
target_link_libraries(map_publisher map_loader ${catkin_LIBRARIES})