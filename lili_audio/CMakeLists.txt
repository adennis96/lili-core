cmake_minimum_required(VERSION 2.8.3)
project(lili_audio)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES RecognizeSpeech.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
