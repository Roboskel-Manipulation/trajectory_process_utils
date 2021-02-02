cmake_minimum_required(VERSION 3.0.2)
project(offline_trajectory_process)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  trajectory_custom_msgs
  message_generation
)

add_service_files(
  FILES
	Filtering.srv
)

generate_messages(
	DEPENDENCIES
)

catkin_package(
	CATKIN_DEPENDS message_generation roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

