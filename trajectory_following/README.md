# Description

High-level control graph-based algorithms for labyrinth movement on grid cell graph and trajectory followning to follow the paths from those graph algorithms: A* libs,
exploration, optimal pathing to center, etc.

# Optimal Navigation

Tries to find the center of the labyrinth by sending A* paths from the current cell to the goal
cell. Labyrinth is treated as a graph problem with a grid structure (no diagonal connections).

Graph is not explored from the start, walls will be discovered on the go. This requires doing
replanning each cell step. Unknown space is treated as free. First run from the start to the 
goal will take some backtracking to reach the goal. Second run can rely on the partial map
from the first run to get a (probably optimal) path to the goal.

Path execution is sent to the trajectory following node (carrot planner).


# Carrot Planner

Follows a given trajectory (list of poses) by sending movement commands. Uses localization info.

Implementation just tries to turn towards the next Pose (effectively, center of the next cell),
ignores obstacles and map data and just moves there.

Current implementation uses ONLY turn in place and GO forward to use not-PWM API.

## Robot API

TODO

Subscribes to 'goal_path' topic with a List[Pose] info about the trajectory to follow.

Subscribes to 'odom' (Odometry) to get current pose data.

Publishes 'cmd_vel' (Twist) of center of mass (CoM) to follow it.

