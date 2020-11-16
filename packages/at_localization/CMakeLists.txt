cmake_minimum_required(VERSION 2.8.3)
project(at_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  tf
  tf2_ros
)

catkin_package()