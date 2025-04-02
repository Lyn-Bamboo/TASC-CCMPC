# TASC-CCMPC

This project develops a probabilistic trajectory generation algorithm based on distributed model predictive control (MPC) for multi-robot systems, accounting for state estimation noise and motion disturbances. 
Our method enhances collision avoidance by incorporating uncertainties through the time-aware Safe Corridor (TASC) formulation. 
Considering these uncertainties, we establish collision avoidance chance constraints by transforming probabilistic conditions into deterministic constraints on the mean and covariance of robot states.

The project has been tested on Ubuntu 20.04 (ROS noetic). The backend optimizer is Gurobi. Please install the [Gurobi Optimizer](https://www.gurobi.com) (free academic lisence available).

Run this commands:

```bash
git clone https://github.com/Lyn-Bamboo/TASC-CCMPC.git
cd TASC-CCMPC
catkin build
source devel/setup.bash
roslaunch planner swarm.launch   
```

