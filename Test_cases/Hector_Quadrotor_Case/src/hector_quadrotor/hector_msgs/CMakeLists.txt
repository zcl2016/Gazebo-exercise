cmake_minimum_required(VERSION 2.8.3)
project(hector_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Controller_Commands.msg
  Current_Path.msg
  Goal_Info.msg
  Controller_Internals.msg
  State.msg
  State29.msg
  Up_Data_New.msg
  Down_Data_New.msg
  Formation_Type.msg
  Waypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
