# Online Motion Planning

This package implements an online motion planning pipeline for the TurtleBot in ROS 2.

It includes the following nodes:

- `localization_node`: estimates the robot odometry
- `grid_mapping`: builds an occupancy grid map from laser scan data
- `online_motion_planning_node`: computes a path using the RRT* algorithm and publishes velocity commands
- `RViz`: used to visualize the robot, map, and planned path, and to send a goal pose

## How to Run

Step 1: 

```bash
ros2 launch turtlebot_simulation turtlebot_circuit_hoi1.launch.py
```

Step 2: 

```bash
ros2 launch online_motion_planning online_motion_planning.launch.py
```