cmake_minimum_required(VERSION 2.8.3)
project(lili_audio)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  actionlib_msgs
  actionlib
)

add_service_files(
  DIRECTORY srv
  FILES RecognizeSpeech.srv
)

add_action_files(
  DIRECTORY action
  FILES TTS.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
