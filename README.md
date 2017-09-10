CarND-Path-Planning-Project
===========================


My solution is based on the Udacity project walkthrough (in particular this applies to the trajectory generator).

The code is structured into three parts.

1. Path planner
2. State machine
3. Trajectory generator

Path planner
------------

The path planner looks 16 seconds into the future and creates a maze using the cars current position and velocity as starting point. Using the cars current speed

```
----------------
|    |    |    |
|    |    |    | t = 16
----------------
|    |    |    |
|    |    |    | t = 14
----------------
|    |    |    |
|    |    |    | t = 12
----------------
|    |    |    |
|    |    |    | t = 10
----------------
|    |    |    |
|    |    |    | t = 8
----------------
|    |    |    |
|    |    |    | t = 6
----------------
|    |    |    |
|    |    |    | t = 4
----------------
|    |    |    |
|    |    |    | t = 2, S = S0 + v * t
----------------
|    |    |    |
|    |    |    | t0 = 0, S = S0
----------------
```

The for each car detected by the sensors I calculate from each cars current position (s, d) and speed where each car is for every given point of time. If they happen to end up in the grid that grid is marked as an obstacle (X). A car close by with a similar velocity will take up many spaces in the grid.

The starting point (C) is the grid space at t=0 and the lane the car currently occupies. The goal (G) for the path planner is set to the most distant grid row but at the same lane. This setup favors staying in a straight line if no other cars are present.


```
----------------
|    |    |    |
| G  |    |    | t = 16
----------------
|    |    |    |
|    |    | X  | t = 14
----------------
|    |    |    |
|    |    | X  | t = 12
----------------
|    |    |    |
| X  |    | X  | t = 10
----------------
|    |    |    |
| X  |    |    | t = 8
----------------
|    |    |    |
| X  |    |    | t = 6
----------------
|    |    |    |
|    |    |    | t = 4
----------------
|    |    |    |
|    |    | X  | t = 2, S = S0 + v * t
----------------
|    |    |    |
| C  |    | X  | t0 = 0, S = S0
----------------
```

This grid and obstacles is fed into an A* solver. I opted for using an existing implementation (https://github.com/daancode/a-star).

```
----
|    |    |    |
| G  |    |    | t = 16
----------------
|    |    |    |
| 1  |    | X  | t = 14
----------------
|    |    |    |
| 2  | 3  | X  | t = 12
----------------
|    |    |    |
| X  | 4  | X  | t = 10
----------------
|    |    |    |
| X  | 5  |    | t = 8
----------------
|    |    |    |
| X  | 6  |    | t = 6
----------------
|    |    |    |
| 8  | 7  |    | t = 4
----------------
|    |    |    |
| 9  |    | X  | t = 2, S = S0 + v * t
----------------
|    |    |    |
| C  |    | X  | t0 = 0, S = S0
----------------
```

The path planner uses only the first step in the solution (marked 9 above) to make it's path recommendation. In this case it recommends to keep the current lane. The path planner results is printed to standard out.

There are certainly possible optimizations in this module. The path planner could explore the velocity dimension and try to find solutions when slowing down or accelerating from the current speed. To avoid collisions i set a two second margin (in relation to the car speed) to other cars. If this margin could be lowered the car would be much better to navigate queues. That would require trying to predict the other cars movements more in detail.

State machine (Should probably be named vehicle)
------------------------------------------------

The result of the path planner is fed into the state machine as either (KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT).

The state machine will update it's internal state and take the path planner's result into account depending on the current state. The state machine will only honor the path plan if it's in a KEEP_LANE state. The state machine adjusts the speed according to traffic in front of it.

Trajectory generator
--------------------

Based on the state (desired lane and speed) the trajectory generator generates a trajectory using the spline library. To avoid jerk and to much acceleration the waypoints spacing is linear to the current speed. I settled for this simplified model as it turned out to work out really well.
