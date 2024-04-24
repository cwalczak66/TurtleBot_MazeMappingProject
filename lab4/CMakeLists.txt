cmake_minimum_required(VERSION 2.8.3)
project(lab4)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs
  std_srvs
)

find_package(catkin REQUIRED
  turtlebot3_description
  turtlebot3_bringup
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################


add_service_files(
  FILES
  Cspace.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   nav_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RBE3002_template
   CATKIN_DEPENDS rospy turtlebot3_bringup turtlebot3_description
#  DEPENDS system_lib
)







###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


