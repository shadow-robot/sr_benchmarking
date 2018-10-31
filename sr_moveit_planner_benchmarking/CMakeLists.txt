cmake_minimum_required(VERSION 2.8.3)
project(sr_moveit_planner_benchmarking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(sr_moveit_run_benchmark src/SrRunBenchmark.cpp)
target_link_libraries(sr_moveit_run_benchmark ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
