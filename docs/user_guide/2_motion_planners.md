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
Currently only the OMPL planners are well integrated in MoveIt!, so we have only used those in our benchmarks.
The following is the set of OMPL planners selected (Description taken form [OMPL website](http://ompl.kavrakilab.org/planners.html)):

* Single-query planners
  * Rapidly-exploring Random Trees
    * **RRT**: A well-known Sample Based Planning algorithm, however the plans generated with it are not optimal.
    * **RRT***: A variation of RRT with proven asymptotically optimal property at the cost of execution time and slower path convergence rate.
    * **[T-RRT](http://www.leonardjaillet.com/Publications_files/Iros08_Jaillet_TransitRRT.pdf)** (Transition-based RRT): It combines the exploration strength of the RRT algorithm that rapidly grow random trees toward unexplored regions of the space, with the efficiency of stochastic optimization methods that use transition tests to accept or to reject a new potential state. It does not give any hard optimality guarantees, but tries to find short, low-cost paths.
    * **RRT Connect**: It works by incrementally building two rapidly-exploring random trees (RRTs) rooted at the start and the goal configurations.
    * **BiTRRT**
  * **EST** (Expansive Space Trees)
  * **BiEST**: Bidirectional version
  * **ProjEST**: Projection-based version
  * **[SBL](http://robotics.stanford.edu/~latombe/papers/isrr01/spinger/latombe.pdf)** (Single-query, Bi-directional and Lazy Collision Checking)
  * **KPIECE** (Kinematic Planning by Interior-Exterior Cell Exploration): KPIECE is a tree-based planner that uses a discretization (multiple levels, in general) to guide the exploration of the (continuous) state space. OMPL's implementation is a simplified one, using a single level of discretization: one grid. The grid is imposed on a projection of the state space. When exploring the space, preference is given to the boundary of that part of the grid that has been explored so far. The boundary is defined to be the set of grid cells that have fewer than 2n non-diagonal non-empty neighboring grid cells in an n-dimensional projection space.
  * **BKPIECE** (Bidirectionnal KPIECE)
  * **LBKPIECE** (Lazy BKPIECE)
  * **Fast Marching Tree algorithm (FMT)**: The FMT∗ algorithm performs a “lazy” dynamic programming recursion on a set of probabilistically-drawn samples to grow a tree of paths, which moves outward in cost-to-come space. Unlike all other planners, the numbers of valid samples needs to be chosen beforehand.
  * **Path-Directed Subdivision Trees (PDST)**: PDST is a planner that has entirely removed the dependency on a distance measure, which is useful in cases where a good distance metric is hard to define. PDST maintains a binary space partitioning such that motions are completely contained within one cell of the partition. The density of motions per cell is used to guide expansion of the tree.
  * **Search Tree with Resolution Independent Density Estimation (STRIDE)**: This planner was inspired by EST. Instead of using a projection, STRIDE uses a Geometric Near-neighbor Access Tree to estimate sampling density directly in the state space. STRIDE is a useful for high-dimensional systems where the free space cannot easily be captured with a low-dimensional (linear) projection.
* Multi-query planners
  * **[PRM](https://ieeexplore.ieee.org/document/508439)** (Probabilistic roadmap): It takes random samples from the configuration space of the robot, testing them for whether they are in the free space, and use a local planner to attempt to connect these configurations to other nearby configurations. The starting and goal configurations are added in, and a graph search algorithm is applied to the resulting graph to determine a path between the starting and goal configurations.
  * **PRM***: While regular PRM attempts to connect states to a fixed number of neighbors, PRM* gradually increases the number of connection attempts as the roadmap grows in a way that provides convergence to the optimal path.
  * **LazyPRM**: This planner is similar to regular PRM, but checks the validity of a vertex or edge “lazily,” i.e., only when it is part of a candidate solution path.
  * **LazyPRM***: A version of PRM* with lazy state validity checking.
  * **SPArse Roadmap Spanner algorithm (SPARS)**: SPARS is a planner that provides asymptotic near-optimality (a solution that is within a constant factor of the optimal solution) and includes a meaningful stopping criterion. Although it does not guarantee optimality, its convergence rate tends to be much higher than PRM*.
  * **SPARS2**: SPARS2 is variant of the SPARS algorithm that works through similar mechanics, but uses a different approach to identifying interfaces and computing shortest paths through said interfaces.

```eval_rst
.. Note:: There have been recent updates in MoveIt! that allow the concept of Planning Request Adapters that is used to
   enable the usage of multiple planning algorithms to be used together in MoveIt. This enables the user to use motion
   planning algorithms in a pipeline to produce better trajectories in different situations. It would be nice to test this
   funcionallity with our benchmarks in the future. More information can be found `here <https://discourse.ros.org/t/updates-for-motion-planners-in-moveit/6765>`_.
```
