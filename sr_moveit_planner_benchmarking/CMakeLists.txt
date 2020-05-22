cmake_minimum_required(VERSION 2.8.3)
project(sr_moveit_planner_benchmarking)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  moveit_ros_benchmarks
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    moveit_ros_benchmarks
  INCLUDE_DIRS include
)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(sr_moveit_run_benchmark src/SrRunBenchmark.cpp src/SrBenchmarkExecutor.cpp)
target_link_libraries(sr_moveit_run_benchmark ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp")

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
# if (CATKIN_ENABLE_TESTING)
#     find_package(rostest REQUIRED)
#     add_rostest(test/test_import_export_scenes.test)
#     add_rostest(test/test_import_export_queries.test)
# endif ()