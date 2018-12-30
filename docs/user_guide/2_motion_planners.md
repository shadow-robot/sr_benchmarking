# Motion Planners available in MoveIt!

## Currently available in MoveIt!
Currently MoveIt! provide the following planners:

* **[Open  Motion  Planning  Library](http://ompl.kavrakilab.org/)** (OMPL) is  a  popular  choice to solve a motion problem. It is an open-source motion 
  planning  library  that  houses  many  state-of-the-art  sampling  based motion planners. OMPL is configured as the default 
  set of planners for MoveIt!. Currently 23 sampling-based motion planners can be selected for use, and they are described [here](http://ompl.kavrakilab.org/planners.html). 

* **[Stochastic  Trajectory Optimization for Motion Planning](http://wiki.ros.org/stomp_motion_planner)** (STOMP)  is  an  
  optimization-based  motion  planner.  It  is designed  to  plan  smooth  trajectories  for  robotic  arms, avoiding obstacles, and optimizing constraints. 
  The  planner is currently partially supported in MoveIt!

* **[Covariant Hamiltonian Optimization for Motion Planning](https://www.ri.cmu.edu/pub_files/2009/5/icra09-chomp.pdf)** (CHOMP) 
  this algorithm capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design 
  a motion planning algorithm based entirely on trajectory optimization. Given an infeasible naive trajectory, CHOMP reacts  
  to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamical 
  quantities such as joint velocities and accelerations. It rapidly converges to a smooth collision-free trajectory that can
  be executed efficiently on the robot. The  planner is currently partially supported in MoveIt!

* **[Search-Based Planning Library](http://wiki.ros.org/sbpl)** (SBPL) consists of a set of planners using search-based planning that discretize the space. 
  Integration into latest version of MoveIt! is work in progress. 

## Selected planners for our benchmarks
As currently, only the OMPL planners are well integrated in MoveIt!, we have only use those in our benchmarks. 
The following is the set of OMPL planners selected:

* Single-query planners
  * **RRT (Rapidly-exploring Random Trees)**: A well-known Sample Based Planning algorithm, however the plans generated with it are not optimal.
  * **RRT***: A variation of RRT with proven asymptotically optimal property at the cost of execution time and slower path convergence rate.
  * **RRT Connect**: It works by incrementally building two rapidly-exploring random trees (RRTs) rooted at the start and the goal configurations.
  * **[T-RRT](http://www.leonardjaillet.com/Publications_files/Iros08_Jaillet_TransitRRT.pdf)** (Transition-based RRT): It combines the exploration strength of the RRT algorithm that rapidly grow random trees toward unexplored regions of the space, with the efficiency of stochastic optimization methods that use transition tests to accept or to reject a new potential state. It does not give any hard optimality guarantees, but tries to find short, low-cost paths.
  * **EST** (Expansive Space Trees)
  * **[SBL](http://robotics.stanford.edu/~latombe/papers/isrr01/spinger/latombe.pdf)** (Single-query, Bi-directional and Lazy Collision Checking)
  * **KPIERCE**
  * **KPIERCE** (Bidirectionnal KPIERCE)
  * **LBKPIERCE** (Lazy BKPIERCE)

* Multi-query planners
  * **[PRM](https://ieeexplore.ieee.org/document/508439)** (Probabilistic roadmap): It takes random samples from the configuration space of the robot, testing them for whether they are in the free space, and use a local planner to attempt to connect these configurations to other nearby configurations. The starting and goal configurations are added in, and a graph search algorithm is applied to the resulting graph to determine a path between the starting and goal configurations.
  * **PRM***: While regular PRM attempts to connect states to a fixed number of neighbors, PRM* gradually increases the number of connection attempts as the roadmap grows in a way that provides convergence to the optimal path.
  

```eval_rst
.. Note:: There has been resent updates in MoveIt! that allow the concept of Planning Request Adapters that is used to
   enable the usage of multiple planning algorithms to be used together in MoveIt. This enables the user to use motion 
   planning algorithms in a pipeline to produce better trajectories in different situations. It would be nice to test this 
   funcionallity with our benchmarks in the future. More information can be found `here <https://discourse.ros.org/t/updates-for-motion-planners-in-moveit/6765>`_.
```