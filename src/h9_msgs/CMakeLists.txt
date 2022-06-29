cmake_minimum_required(VERSION 2.8.3)
project(h9_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  ABS1.msg
  ABS2.msg
  ABS3.msg
  EMS2.msg
  EMS3.msg
  GPS1.msg
  SAS1.msg
  TCU3.msg
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
