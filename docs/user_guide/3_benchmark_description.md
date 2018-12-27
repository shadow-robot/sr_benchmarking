# Benchmark description
The benchmark problems we define here require the specification of these components: 
* Robot model: The particular robot used for the experiments.
* Scenes: The environment to be used for motion planning.
* Queries: A set of initial and goal robot states associated with the scenes.

## Robot Model


## Scenes
The scene file format (.scene) is used to define a scene where the robot will be performing the
motions. It can be imported, modified and exported using the Motion Planning RViz
plugin. 

The scenes defined for our benchmarks can be found [here](https://github.com/shadow-robot/sr_benchmarking/tree/kinetic-devel/sr_moveit_planner_benchmarking/experiments/scenes) 
and are shown in the following pictures:

```eval_rst
+------------+------------+
| |image0|   | |image1|   |
+============+============+
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


## Benckmark configuration

## Benchmark metrics

## Tools to work with the queries and scenes files 

