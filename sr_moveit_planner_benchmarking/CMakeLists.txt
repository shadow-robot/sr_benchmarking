cmake_minimum_required(VERSION 2.8.3)
project(sr_moveit_planner_benchmarking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_ros_benchmarks
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    moveit_ros_benchmarks
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(sr_benchmark_executor src/SrBenchmarkExecutor.cpp)
target_link_libraries(sr_benchmark_executor ${catkin_LIBRARIES})

add_executable(sr_moveit_run_benchmark src/SrRunBenchmark.cpp src/SrBenchmarkExecutor.cpp)
target_link_libraries(sr_moveit_run_benchmark ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
