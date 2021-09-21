cmake_minimum_required(VERSION 3.0.2)
project(rikuken_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf 
  tf2
  gazebo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rikuken_robot
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(model_tf_publisher src/model_tf_publisher_node.cpp src/model_tf_publisher.cpp)
target_link_libraries(model_tf_publisher
  ${catkin_LIBRARIES}
)