cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot_simulation turtlebot_hoi_circuit1.launch.py

cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch online_motion_planning online_motion_planning.launch.py

cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot_simulation turtlebot_hoi_circuit1.launch.py

cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot_simulation turtlebot_hoi_circuit2.launch.py

cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot_simulation turtlebot_hoi_circuit2_closemod.launch.py

cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch online_motion_planning online_motion_planning.launch.py


cd /home/elchina/Documents/HOP_project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select online_motion_planning
source install/setup.bash









- `min_goal_distance_cells`: Minimum robot-to-goal distance in grid cells; avoids tiny/oscillatory goals.
- `goal_reached_distance_m`: Distance threshold (meters) to consider current goal reached.

- `free_threshold`: Max occupancy value treated as free.
- `unknown_low`, `unknown_high`: Occupancy value range treated as unknown.
- `occupied_threshold`: Min value in inflated map treated as occupied/blocking.

- `goal_clearance_cells`: Required obstacle-free buffer (cells) around candidate goal.
- `border_margin_cells`: Reject goals too close to map borders (artifact-prone zone).
- `unknown_clearance_radius`: Reject goals if unknown cells exist within this neighborhood radius.
- `inward_goal_offset_cells`: Shift goal inward from frontier toward safer known space.

- `active_goal_world`: Current active goal in world coordinates.
- `active_goal_cell`: Current active goal in grid coordinates.
- `active_goal_time`: Timestamp when current goal was issued.
- `goal_timeout_sec`: Time limit before abandoning current goal.
- `goal_start_robot_xy`: Robot position when current goal started (for progress check).
- `progress_check_sec`: Time window before checking if robot made enough progress.
- `min_progress_m`: Minimum required progress (meters) toward goal in that window.

- `goal_blacklist`: Temporarily blocked goal cells/regions after failures.
- `blacklist_duration_sec`: How long blocked cells stay blacklisted.
- `blacklist_radius_cells`: Radius (cells) of region blacklisted around failed goal.