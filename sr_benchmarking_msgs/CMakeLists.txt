cmake_minimum_required(VERSION 2.8.3)
project(sr_benchmarking_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  message_generation
  geometry_msgs
  std_msgs
)

add_service_files(
   FILES
   ExecuteBenchmarking.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)
