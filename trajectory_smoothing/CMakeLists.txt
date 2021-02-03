cmake_minimum_required(VERSION 3.0.2)
project(trajectory_smoothing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  trajectory_custom_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Smoothing.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

catkin_package(
 CATKIN_DEPENDS message_generation roscpp rospy 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
