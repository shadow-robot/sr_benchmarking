cmake_minimum_required(VERSION 2.8.3)
project(sr_benchmarking)

find_package(catkin REQUIRED
  genmsg
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ExecuteBenchmarking.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sr_benchmarking
  CATKIN_DEPENDS message_runtime rospy std_msgs
  DEPENDS python-psutil
)

# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sr_benchmarking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
