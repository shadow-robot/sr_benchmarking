# Benchmark description
The benchmark problems we define here require the specification of these components: 
* Robot model: The particular robot used for the experiments.
* Scenes: The environment to be used for motion planning.
* Queries: A set of initial and goal robot states associated with the scenes.

## Robot Model

For our benchmarks, we used the Universal Robots UR10 robot arm. We attached a box as end-effector to simulate the collision issues 
we will have if a robot hand was attached. The MoveIt! configuration used can be found [here](https://github.com/shadow-robot/sr_interface/tree/kinetic-devel/sr_multi_moveit/sr_box_ur10_moveit_config).

The robot can be launched using the following command:
```bash
roslaunch sr_moveit_planner_benchmarking robot.launch
``` 

You can add the following arguments:
* initial_z: to move the robot up, such as when it needs to be on the table
* visualization: if it is set to true, rviz will be launched otherwise it will not, which is sometimes desired.
* robot_description: you can change the urdf used in case you want to change your robot or change some urdf values.
 

## Scenes
The scene file format (.scene) is used to define a scene where the robot will be performing the
motions. It can be imported, modified and exported using the Motion Planning RViz
plugin. 

The scenes defined for our benchmarks can be found [here](https://github.com/shadow-robot/sr_benchmarking/tree/kinetic-devel/sr_moveit_planner_benchmarking/experiments/scenes) 
and are shown in the following pictures:
```eval_rst
+------------+------------+
| |image0|   | |image1|   |
+------------+------------+
| |image2|   | |image3|   |
+------------+------------+
| |image4|   | |image5|   |
+------------+------------+

.. |image0| image:: ../img/Scene_ground.png
.. |image1| image:: ../img/Scene_ground_with_boxes.png
.. |image2| image:: ../img/Scene_table.png
.. |image3| image:: ../img/Scene_table_with_objects.png
.. |image4| image:: ../img/Scene_table_with_tote.png
.. |image5| image:: ../img/Scene_table_with_two_totes.png

```

## Queries

A query is defined as the start and goal state for a robot in a  specific scene. We have used Rviz to 
create a set of queries for each scene with different complexity levels. The queries for each scene can be found [here](https://github.com/shadow-robot/sr_benchmarking/tree/kinetic-devel/sr_moveit_planner_benchmarking/experiments/queries).

## Benckmark configuration

Each benchmark has a configuration file in yaml format, containing the following information:
* Details of the warehouse
* Number of runs for each planning algorithm on each request
* The name of the group to plan
* The maximum time for a single run
* The directory to write the output to
* The stored start states in the warehouse to try
* The motion plan queries in the warehouse to try
* The bounds of the workspace the robot plans in.
* A list of planners to benchmark the queries in.

This is an example of the format of a benchmark configurarion yaml file:
```
benchmark_config:
    warehouse:
        host: 127.0.0.1
        port: 33829
        scene_name: scene_ground
    parameters:
        name: scene_ground
        runs: 10
        group: right_arm
        timeout: 10.0
        output_directory: /tmp/moveit_benchmarks/
        queries: .*
    planners:
        - plugin: ompl_interface/OMPLPlanner
          planners:
            - SBLkConfigDefault
            - ESTkConfigDefault
            - LBKPIECEkConfigDefault
            - BKPIECEkConfigDefault
            - KPIECEkConfigDefault
            - RRTkConfigDefault
            - RRTConnectkConfigDefault
            - RRTstarkConfigDefault
            - TRRTkConfigDefault
            - PRMkConfigDefault
            - PRMstarkConfigDefault
```

The configuration for our benchmarks can be found [here](https://github.com/shadow-robot/sr_benchmarking/tree/kinetic-devel/sr_moveit_planner_benchmarking/experiments/benchmark_configs).

## Tools to work with the queries and scenes files 

To load the .scene and .queries files into the warehouse, you can use the following methods:

### Our high level methods

* To load all our scene and queries to the warehouse:
```bash
roslaunch sr_moveit_planner_benchmarking load_all_scenes_and_queries_to_db.launch
``` 

* To load one scene and queries files:
```bash
roslaunch sr_moveit_planner_benchmarking load_scenes_and_queries_to_db.launch queries_file:=[path_to_file] scene_file:=[path_to_file]
``` 

* To load just a scene:
```bash
roslaunch sr_moveit_planner_benchmarking load_scenes_to_db.launch scene_file:=[path_to_file]
``` 

* To load just a query:
```bash
roslaunch sr_moveit_planner_benchmarking load_queries_to_db.launch queries_file:=[path_to_file]
``` 

* To export a scene that is in the warehouse to a .scene text file:
```bash
roslaunch sr_moveit_planner_benchmarking export_scenes_to_text.launch output_directory:=[path_to_folder_to_save_file]
``` 

* To export the queries that is in the warehouse to a .queries text file:
```bash
roslaunch sr_moveit_planner_benchmarking export_queries_to_text.launch output_directory:=[path_to_folder_to_save_file] (group_prefix:=ra) (cartesian:=false)
``` 
If you want to export the queries in cartesian space (position and orientation) instead of joint space, you can set the cartesian argument to true.

### Methods from MoveIt!
The previous methods, internally call a set of MoveIt! methods. You can call the MoveIt! methods directly. 

#### Export to text

An executable is available to allow to export the scene and queries that are available in the Moveit! warehouse to text files:

```
rosrun moveit_ros_warehouse moveit_warehouse_save_as_text
```

It has different parameters that can be specified:

* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `output_directory`: Directory to save the generated files
* `scene`: Saves the scenes available in the warehouse. It generates .scene files. If it is not specified the queries are saved.
* `cartesian`: Save queries in cartesian space (start and end pose of eef). If not specified they will be saved in joint space by default.
* `eef`: Specify the end effector (Only needed when the cartesian option is set). Default: last link. 
* `group_prefix`: Specify the group prefix you'd like to plan with. This is useful if you want to save the queries only for the arm and it has a prefix, so only those joints will be considered (e.g. "ra")

To be able to use this command, the robot should be launched in another terminal first.

Here are a few examples of how to use them:
* Example to export the scenes:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --scene --output_directory /tmp/scene
  ```
  Example of the exported scene (empty space with ground): example_scene.scene
  ```
  example_scene
  * ground__link
  1
  box
  5 5 0.01
  -0.2 0.6 0
  0 0 0 1
  0 0 0 0
  .
  ```
* Example to export the queries in joint space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra
  ```
  Example of exporting a query: example_scene.queries
  ```
  example_scene
  example_query
  START
  ra_shoulder_pan_joint = -0.000150601
  ra_shoulder_lift_joint = -0.992381
  ra_elbow_joint = 1.73153
  ra_wrist_1_joint = 2.46467
  ra_wrist_2_joint = -0.000163041
  ra_wrist_3_joint = 3.07979
  .
  GOAL
  joint_constraint
  ra_shoulder_pan_joint = 2.20941
  ra_shoulder_lift_joint = -1.89185
  ra_elbow_joint = 1.81825
  ra_wrist_1_joint = 0.14342
  ra_wrist_2_joint = 2.20853
  ra_wrist_3_joint = 0.0420439
  .
  ```
* Example to export the queries in cartesian space:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_save_as_text --output_directory /tmp/queries --group_prefix ra --cartesian --eef ra_wrist_3_link
  ```
  Example of exporting a query: cartesian_example_scene.queries
  ```
  example_scene
  cartesian_query
  START
  ra_shoulder_pan_joint = 2.64932
  ra_shoulder_lift_joint = -1.99619
  ra_elbow_joint = -1.29736
  ra_wrist_1_joint = 1.87004
  ra_wrist_2_joint = -2.11698
  ra_wrist_3_joint = 2.63777
  .
  GOAL
  position_constraint
  End_effector = ra_wrist_3_link
  Position = -0.317122 -0.130014 0.548275
  Orientation = 0.0291105 0.158759 0.383943 0.90914
  ```
  
#### Import from text

An executable is available to allow to import .scenes and .queries files to the Moveit! warehouse:

```
rosrun moveit_ros_warehouse moveit_warehouse_import_from_text
```

It has different parameters that can be specified:
* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `queries`: Name of file containing motion planning queries.
* `scene`: "Name of file containing motion planning scene."

To be able to use this command, the robot should be launched in another terminal first.

Here are a few examples of how to use them:
* Example to import a scene file:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_import_from_text --scene example_scene.scene
  ```
* Example to import a query file:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_import_from_text --queries example_scene.queries
  ```

#### Generate random valid queries

An executable is available to allow to the generation of a specified number of valid (collision free) random queries and save them into the Moveit! warehouse:

```
rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries [name_of_scene] [number_of_random_queries]
```

It has different parameters that can be specified:
* `help`: Show help message
* `host`: Host for the DB. Default 127.0.0.1.
* `port`: Port for the DB. Default 33829.
* `limited_joints`: Limit joints from -pi to pi to avoid a lot of impossible queries
* `group_prefix`: Specify the group prefix you'd like to plan with. This is useful if you want to save the queries only for the arm and it has a prefix, so only those joints will be considered (e.g. "ra")
* `cartesian`: Generate the cartesian space query equivalent to the one generated in joint space
* `eef`: Specify the end effector (Only needed when the cartesian option is set). Default: last link. 
* `clear`: Clears all the random queries for a given scene

After generating the queries, they are saved in the warehouse. You can use the commands above to export them to text.

Here are a few examples of how to use them:
* Example of generating 10 random queries in joint space in the example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra
  ```
* Example of generating 10 random queries in joint space in the example_scene limiting the joints:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra --limited_joints
  ```
* Example of generating 10 random queries in cartesian space in the example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene 10 --group_prefix ra --cartesian --eef ra_wrist_3_link
  ```
* Example of clearing the random queries in example_scene:
  ```
  rosrun moveit_ros_warehouse moveit_warehouse_generate_random_queries example_scene --clear


## Benchmark metrics

We use several metrics to perform the benchmarks of the different planneres.

There are the metrics defined by default in MoveIt!:
* **Total time (s):**<br>
  Time taken by the whole process. It is calculated with the following formula: <br>
  T = plan_time  + interpolation_time + simplify_time + process_time <br>
  The lower the value, the better performance of the planner. 
* **Solved (%):**<br>
  Percentage of queries that the planner was able to find a solution for. <br>
  The higher the value, the better performance of the planner.  
* **Lenght (rad):**<br>
  Calculated by a sum of angles traveled by each of the joints. Formula: <br>
  L = sum<sub>i=0</sub><super>n-1</super>{abs(x<sub>i</sub>- x<sub>i0</sub>)}, where: <br>
  n - number of robot's joints, <br>
  x - joint's goal position, <br>
  x<sub>0</sub> - joint's initial position. <br>
  The lower the value, the better the plan.
* **Clearance (m):**<br>
  Calculated by average distance to nearest invalid state (obstacle) throughout the planned path. Formula: <br>
  C = (1/n)<sub> * </sub>sum<sub>i=0</sub><super>n-1</super>{cl(s<sub>i</sub>)}, where: <br>
  C = (1/n) * sum<sub>i=0</sub><super>n-1</super>{cl(s<sub>i</sub>)}, where: <br>
  n - number of states on path,<br>
  s<sub>i</sub> - ith state,<br>
  cl() - distance to the first invalid state.<br>
  The higher the value, the better the plan.
