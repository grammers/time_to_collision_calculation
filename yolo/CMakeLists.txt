cmake_minimum_required(VERSION 2.8.3)
project(yolo_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  jsk_recognition_msgs
  #geometry_msgs
  #message_generation
)

#add_message_files(
#    FILES
#    BoundingBox.msg
#    BoundingBoxArray.msg
#)

#generate_messages()
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)
## Build ##


###########
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


