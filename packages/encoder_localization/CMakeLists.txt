cmake_minimum_required(VERSION 2.8.3)
project(encoder_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  duckietown_msgs
  message_generation
)

add_service_files(
  FILES
  InitFrame.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()