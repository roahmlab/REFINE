cmake_minimum_required(VERSION 3.0.2)
project(state_estimator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  message_generation
  rover_control_msgs
)

add_message_files(
  DIRECTORY msg
  FILES velocity_states.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS rover_control_msgs geometry_msgs nav_msgs rospy std_msgs message_runtime
)
