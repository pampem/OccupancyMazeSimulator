# Occupancy Maze Simulator

This package allows you to test your path planning node by providing a maze-based occupancy grid message and a simulated robot pose response. It includes random maze generation, start and goal positioning, and interactive feedback with Twist commands for movement.
## Example Usage

```bash
rviz2 -d occupancy_maze_simulator/rviz/config.rviz 

ros2 run occupancy_maze_simulator occupancy_maze_simulator_node \
--ros-args \
--param obstacle_mode:=maze \
--param gridmap.resolution:=1.0 \
--param gridmap.x:=50.0 \
--param gridmap.y:=50.0 \
--param start_position:=[2,2] \
--param goal_position:=[48,48] \
--param maze.density:=0.1
```

obstacle_mode: maze or random

![poem](doc/media/poemByCodeRabbit.png)