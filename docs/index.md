<img src="https://www.shadowrobot.com/wp-content/uploads/copy-Shadow-Logo.png" />

# Benchmarking motion planners available in Moveit!

```eval_rst
.. image:: img/scene_table_with_tote_rrtconnect.gif
```

At [Shadow Robot](https://www.shadowrobot.com/), we use the Universal Robot Arms (UR5 and UR10) with our robot hands
to perform experiments in order to improve our robot manipulation capabilities. However, a
challenge remains to find accurate and reliable plans to move the arm around our
environments even for simple movements. In most cases, the planner algorithms failed to
find a plan in a reasonable amount of time or the plan generated makes the robot perform
unnecessary movements that might be dangerous to execute for the real arm.

Motion planning is a common problem in robotics and even for the simple problem of finding
a path connecting two states while avoiding collisions, there is no efficient solution for the
general case. This inspired us to provide the tools to
perform rigorous testing of the different planners available and to provide researchers tools
to help them find solutions for their specific problems.

We use the well-know motion planning framework [MoveIt!](https://moveit.ros.org/). MoveIt! itself does not provide  
motion planning, but instead is designed to work with planners or planning libraries. 

Our base is the [MoveIt! ROS Benchmarks package](https://github.com/ros-planning/moveit/tree/kinetic-devel/moveit_ros/benchmarks) 
which provides methods to benchmark motion planning algorithms and aggregate/plot statistics using the OMPL Planner Arena.
Here you can follow [an example](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/benchmarking_tutorial.html) 
in which they demonstrate how the benchmarking can be run for a Fanuc M-10iA.

We have proposed a series of benchmark problems that are designed to exercise the planners in different scenarios focused 
specially on common manipulation problems. Different set of queries were defined in each scenario with difficult levels of 
difficulty with the intention of testing the performance of the planners. Additionally a set of valid random queries were 
generated in each scenario to validate the results of each planner. 

These sets of benchmark problems intend to help the community to be a stating point and we encurage the scientific community
to contribute with more scenes and queries that exwecise other capabilities of planners.



## Contents

* [Installing the software](user_guide/1_installing_the_software.md)
* [Motion planners](user_guide/2_motion_planners.md)
* [Benchmarks description](user_guide/2_benchmark_description.md)
* [Running an experiment](user_guide/3_running_an_experiment.md)
* [Visualizing the results](user_guide/4_visualizing_the_results.md)