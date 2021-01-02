# Holonomic-MPC---ORCA

## Introduction

The MPC has a prediction horizon of N steps and planning frequency of M steps.

Use run_mpc_2agents.m to run the file.

Provide desired start and goal poses in the agents struct.

Adjust the N,M and dt parameters accordingly.

You have to provide a start and goal pose to obtain the collision avoidance trajectories.


## Costs and Constraints

We have acceleration constraints to maintain the bound the acceleration values.

comp_mpc.m is the file that computes the optimization problem

    line 3 - call to the cost function(Goal Reaching , Smoothness Cost and Control Cost)
 
    lines 102 - 112 - call to obtain ORCA constraints parameters(getorca_lin.m)
 
          113 - 120 - Forming the linear ORCA constraints
       
    line 134 - Non Linear Collision Avoidance RVO constraint(rvo_hol.m)   
    
getorca_lin.m is a direct implementation of ORCA constraints by forming a Collision cone and outputs the parameters "u" and normal vector to bring the velocity outside the Velocity Obstacle.

rvo-hol.m - Using rvo nonlinear constraints

    line 29 - RVO constraint 
    
    


 
       
