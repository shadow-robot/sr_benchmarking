cmake_minimum_required(VERSION 2.8.3)
project(sr_benchmarking)

find_package(catkin REQUIRED COMPONENTS rospy)
include_directories(${Boost_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
  DEPENDS python-psutil
  CATKIN_DEPENDS rospy
  INCLUDE_DIRS
  LIBRARIES
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
