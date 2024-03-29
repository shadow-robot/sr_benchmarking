# Running a benchmarking experiment

## Executing the benchmarks

After defining the scenes, queries and benchmark configurations, you
are ready to execute the benchmarks. To do so, execute the following launch file:

```bash
roslaunch sr_moveit_planner_benchmarking benchmarking.launch bench_opts:=<path_to_benchmark_config_file>
```
You can also specify the initial_z of the robot to move it on top of the table.

We have created launch files for each of our scenes that you can find [here](https://github.com/shadow-robot/sr_benchmarking/tree/kinetic-devel/sr_moveit_planner_benchmarking/experiments/launch).
For example, to run the benchmarks for the scene 'ground_with_boxes', execute this:
```bash
roslaunch sr_moveit_planner_benchmarking benchmark_ground_with_boxes.launch
```

## Generating database files from log files

After the benchmarks are executed, they generate a set of log files (one per query). If you want to convert them to
sqlite3 database files that are easy to handle, run the following command:

* All files
```bash
rosrun sr_moveit_planner_benchmarking sr_moveit_planner_convert_to_db.py -a /tmp/moveit_benchmarks/
```

* Sorted by scenes:
```bash
rosrun sr_moveit_planner_benchmarking sr_moveit_planner_convert_to_db.py -a /tmp/moveit_benchmarks/ --sort
```
