cmake_minimum_required(VERSION 2.8.3)
project(lili_audio)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msg
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INLUDE_DIRS}
)
