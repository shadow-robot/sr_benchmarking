# Running a benchmarking experiment

## Executing the benchmark

## Generating database from log files


To run Moveit benchmarks with our robot:
```
roslaunch sr_moveit_planner_benchmarking benchmarking.launch
```

Convert log files to databases:
* All files
```
rosrun sr_moveit_planner_benchmarking sr_moveit_planner_convert_to_db.py -a results/log_files/
```

* Sorted by scenes:
```
rosrun sr_moveit_planner_benchmarking sr_moveit_planner_convert_to_db.py -a results/log_files/ --sort
```

