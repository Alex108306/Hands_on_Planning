import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray, String
import numpy as np
from tf2_ros import TransformBroadcaster
from online_motion_planning.RRT_star_algorithm import RRT_Star, fill_path, convert_to_path_index
from online_motion_planning.Point import Point

from PIL import Image

from math import atan2
import math

def quaternion_to_yaw(q):
    """Extract yaw (theta) from geometry_msgs quaternion (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class OnlineMotionPlanning(Node):

    def __init__(self):
        super().__init__('online_motion_planning')
        self.declare_parameter('mode', 'sim')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Defining node publisher and subcriber
        self.odom_sub = self.create_subscription(Odometry, "/turtlebot/odom", self.odom_callback, 10)
        self.grid_map_sub = self.create_subscription(OccupancyGrid, "/projected_map", self.grid_map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.vel_pub = self.create_publisher(Twist, '/turtlebot/cmd_vel', 10)
        self.arm_controller_pub = self.create_publisher(
            Float64MultiArray,
            '/turtlebot/swiftpro/joint_velocity_controller/command',
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/inflated_map", 10)
        self.goal_feedback_pub = self.create_publisher(String, "/goal_feedback", 10)
        self.create_timer(1.0, self.replanning_func)
        self.create_timer(0.1, self.controller_func)

        # Initialize map variable
        self.grid_map = None
        self.resolution_map = None
        self.width_map, self.height_map = None, None
        self.origin_map = None

        # Initialize mode-dependent planning/control profile
        if self.mode == 'real':
            self.robot_radius = 0.16
            self.max_linear_velocity = 0.5
            self.max_angular_velocity = 1.5
            self.map_frame = "odom"
            self.use_transposed_grid = True
            # Real robot actuator deadband compensation.
            self.linear_deadband = 0.2
            self.angular_deadband = 0.65
            self.linear_zero_epsilon = 0.02
            self.angular_zero_epsilon = 0.05
        else:
            self.robot_radius = 0.17
            # self.robot_radius = 0.2
            self.max_linear_velocity = 0.4
            self.max_angular_velocity = 2
            self.map_frame = "world_enu"
            self.use_transposed_grid = False
            self.linear_deadband = 0.0
            self.angular_deadband = 0.2
            self.linear_zero_epsilon = 0.0
            self.angular_zero_epsilon = 0.0
        self.kv = 0.5
        self.kw = 0.5

        # Initialize robot pose and goal pose
        self.robot_pose = None
        self.goal_pose = None

        # Initialize RRT* parameters
        self.K = 10000
        self.delta_q = 10
        self.p = 0.2
        self.max_range = 10

        # Path variable
        self.path = None
        self.path_index = 0
        self.rrt_start_seed_search_radius_cells = 16 if self.mode == "real" else 4

        # Initizalize control motion variable
        self.robot_vel = Twist()
        self.look_forward = 0.1

        # One-time startup spin in real mode to reduce LiDAR blind spots.
        self.startup_spin_enabled = self.mode == "real"
        self.startup_spin_done = not self.startup_spin_enabled
        self.startup_spin_target_rad = 2.0 * math.pi
        self.startup_spin_angular_speed = 0.6
        self.startup_spin_accum_rad = 0.0
        self.startup_spin_last_yaw = None
        self.startup_spin_announced = False
        self.startup_spin_completed_announced = False
        self.debug_planner_diagnostics = self.mode == "real"

    def stop_and_clear_navigation_state(self, reason: str):
        """Stop robot and clear path/goal state when navigation is invalid."""
        self.path = None
        self.path_index = 0
        self.goal_pose = None
        # Clear RViz path display as well (avoid stale path confusion).
        empty_path = Path()
        empty_path.header.frame_id = self.map_frame
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)
        self.robot_vel.linear.x = 0.0
        self.robot_vel.angular.z = 0.0
        self.vel_pub.publish(self.robot_vel)
        self.get_logger().info(reason)

    def publish_goal_feedback(self, feedback: str):
        msg = String()
        msg.data = feedback
        self.goal_feedback_pub.publish(msg)

    def apply_deadband_compensation(self, cmd, deadband, zero_epsilon, max_cmd):
        """
        Map low nonzero commands above actuator deadband while preserving true zero.
        """
        mag = abs(cmd)
        if mag <= zero_epsilon:
            return 0.0
        if deadband <= 0.0:
            return float(np.clip(cmd, -max_cmd, max_cmd))
        if deadband >= max_cmd:
            return float(np.sign(cmd) * max_cmd)

        # Linearly map (zero_epsilon..max_cmd) -> (deadband..max_cmd)
        alpha = (mag - zero_epsilon) / max(max_cmd - zero_epsilon, 1e-6)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        mapped_mag = deadband + alpha * (max_cmd - deadband)
        return float(np.sign(cmd) * np.clip(mapped_mag, 0.0, max_cmd))

    def to_index(self, x, y):
        x = int((x - self.origin_map.position.x) / self.resolution_map)
        y = int((y - self.origin_map.position.y) / self.resolution_map)
        return x, y

    def is_valid(self, x, y):
        cell_x, cell_y = self.to_index(x, y)
        # self.get_logger().info('self.grid_map[cell_x, cell_y]: "%s"' % self.grid_map[cell_x, cell_y])
        if self.grid_map[cell_x, cell_y] <= 0.5:
            return True
        else:
            return False

    def find_nearest_rrt_start_seed(self, start_cell, planner_grid):
        sx, sy = int(start_cell[0]), int(start_cell[1])
        width = planner_grid.shape[0]
        height = planner_grid.shape[1]

        def in_bounds(x, y):
            return 0 <= x < width and 0 <= y < height

        def is_free(x, y):
            # planner_grid is binary-like inflated grid (0 free, 1 occupied).
            return planner_grid[x, y] <= 0

        if not in_bounds(sx, sy):
            return None
        if is_free(sx, sy):
            return (sx, sy)

        max_radius = self.rrt_start_seed_search_radius_cells
        for radius in range(1, max_radius + 1):
            best_cell = None
            best_dist_sq = float("inf")
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = sx + dx, sy + dy
                    if not in_bounds(nx, ny):
                        continue
                    if not is_free(nx, ny):
                        continue
                    dist_sq = dx * dx + dy * dy
                    if dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_cell = (nx, ny)
            if best_cell is not None:
                return best_cell

        return None

    def odom_callback(self, msg):
        self.robot_pose = msg
        if not self.startup_spin_done:
            yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            if self.startup_spin_last_yaw is None:
                self.startup_spin_last_yaw = yaw
                return
            dyaw = wrap_to_pi(yaw - self.startup_spin_last_yaw)
            self.startup_spin_accum_rad += abs(dyaw)
            self.startup_spin_last_yaw = yaw
            if self.startup_spin_accum_rad >= self.startup_spin_target_rad:
                self.startup_spin_done = True

    def grid_map_callback(self, msg):
        self.grid_map = msg.data
        self.resolution_map = msg.info.resolution
        self.width_map = msg.info.width
        self.height_map = msg.info.height
        self.origin_map = msg.info.origin

        self.grid_map = np.array(self.grid_map).reshape(self.width_map, self.height_map)/100.0
        self.grid_map = np.where(self.grid_map > 0.5, 1.0, np.where(self.grid_map <= 0.5, 0.0, self.grid_map))

        # Inflate the obstacles in the grid map
        inflation_radius = int(self.robot_radius / self.resolution_map)
        inflated_grid_map = np.copy(self.grid_map)
        for i in range(self.width_map):
            for j in range(self.height_map):
                if self.grid_map[i, j] == 1.0:
                    x_min = max(0, i - inflation_radius)
                    x_max = min(self.width_map, i + inflation_radius + 1)
                    y_min = max(0, j - inflation_radius)
                    y_max = min(self.height_map, j + inflation_radius + 1)
                    inflated_grid_map[x_min:x_max, y_min:y_max] = 1.0
        self.grid_map = inflated_grid_map
        
        inflated_map_msg = OccupancyGrid()
        if self.mode == "real":
            inflated_map_msg.header.frame_id = "odom"
        else:
            inflated_map_msg.header.frame_id = "world_ned"
        inflated_map_msg.info.resolution = self.resolution_map
        inflated_map_msg.info.width = self.width_map
        inflated_map_msg.info.height = self.height_map
        inflated_map_msg.info.origin = self.origin_map
        inflated_map_msg.data = (self.grid_map.flatten() * 100).astype(int).tolist()
        self.inflated_map_pub.publish(inflated_map_msg)


    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        if self.mode == "sim":
            arm_control = Float64MultiArray()
            arm_control.data = [0.0, 0.0, -1.0, 0.0]
            self.arm_controller_pub.publish(arm_control)
        if self.grid_map is None or self.robot_pose is None:
            self.get_logger().info('Waiting for grid map and robot pose to be available...')
            return
        self.planning()
        
    def planning(self):
        if not self.startup_spin_done:
            return False
        # self.get_logger().info('Planning path to goal: "%s" "%s"' % (self.goal_pose.position.x, self.goal_pose.position.y))
        robot_world = (
            self.robot_pose.pose.pose.position.x,
            self.robot_pose.pose.pose.position.y,
        )
        robot_cell = self.to_index(robot_world[0], robot_world[1])
        goal_world = (self.goal_pose.position.x, self.goal_pose.position.y)
        goal_cell = self.to_index(goal_world[0], goal_world[1])
        if self.debug_planner_diagnostics:
            self.get_logger().info(
                "[PLAN_DIAG] mode=%s frame=%s map(w=%d,h=%d,res=%.3f,origin=(%.3f,%.3f)) use_T=%s robot_world=(%.3f,%.3f)->cell=%s goal_world=(%.3f,%.3f)->cell=%s grid_shape=%s"
                % (
                    self.mode,
                    self.map_frame,
                    int(self.width_map),
                    int(self.height_map),
                    float(self.resolution_map),
                    float(self.origin_map.position.x),
                    float(self.origin_map.position.y),
                    str(self.use_transposed_grid),
                    float(robot_world[0]),
                    float(robot_world[1]),
                    str(robot_cell),
                    float(goal_world[0]),
                    float(goal_world[1]),
                    str(goal_cell),
                    str(self.grid_map.shape),
                )
            )

        if self.is_valid(self.goal_pose.position.x, self.goal_pose.position.y):   
            path_msg = Path()
            robot_pose = robot_cell
            goal_pose = goal_cell
            if self.use_transposed_grid:
                planner_grid = self.grid_map.astype(int).T #elchin
            else:
                planner_grid = self.grid_map.astype(int)
            self.grid_map = planner_grid
            if self.debug_planner_diagnostics:
                self.get_logger().info(
                    "[PLAN_DIAG] planner_grid_shape=%s transpose_applied=%s"
                    % (str(planner_grid.shape), str(self.use_transposed_grid))
                )
            start_seed = self.find_nearest_rrt_start_seed(robot_pose, self.grid_map)
            if start_seed is None:
                self.publish_goal_feedback("start_invalid")
                self.get_logger().warn(
                    "RRT start cell is blocked and no nearby free seed was found."
                )
                return False
            if start_seed != (int(robot_pose[0]), int(robot_pose[1])):
                self.get_logger().warn(
                    "RRT start cell is non-traversable; using nearby free seed %s."
                    % (str(start_seed),)
                )
            if self.debug_planner_diagnostics:
                self.get_logger().info(
                    "[PLAN_DIAG] rrt_start=%s rrt_goal=%s"
                    % (str(start_seed), str(goal_pose))
                )
            start = Point(start_seed[0], start_seed[1])
            goal = Point(goal_pose[0], goal_pose[1])
            rrt_star = RRT_Star(grid_map=self.grid_map, K=self.K, delta_q=self.delta_q, p=self.p, max_distance=self.max_range, q_start=start, q_goal=goal)
            path_first_found, path_converge, edge_goal = rrt_star.run_RRT_Star()
            if len(edge_goal) == 0:
                self.publish_goal_feedback("no_path")
                if self.mode == "real":
                    self.get_logger().warn(
                        "No path found; keeping current navigation state in real mode."
                    )
                    return False
                self.stop_and_clear_navigation_state("No path found; stopping and clearing goal.")
                return False
            _, _, path = fill_path(path_converge[0], path_converge[1], edge_goal)
            path_smoothed = rrt_star.smoothing_function(path, path_converge[0])
            self.path = convert_to_path_index(path_converge[0], path_converge[1], path_smoothed)
            self.path_index = 0
            if self.debug_planner_diagnostics:
                total = len(self.path[0]) if self.path is not None else 0
                out_of_bounds = 0
                occupied = 0
                sample_cells = []
                for idx in range(total):
                    cx = int(self.path[0][idx])
                    cy = int(self.path[1][idx])
                    if idx < 5 or idx >= max(0, total - 5):
                        sample_cells.append((cx, cy))
                    if not (0 <= cx < self.grid_map.shape[0] and 0 <= cy < self.grid_map.shape[1]):
                        out_of_bounds += 1
                        continue
                    if self.grid_map[cx, cy] > 0:
                        occupied += 1
                self.get_logger().info(
                    "[PLAN_DIAG] path_points=%d out_of_bounds=%d occupied_cells=%d sample_cells=%s"
                    % (total, out_of_bounds, occupied, str(sample_cells))
                )
            for i in range(len(self.path[0])):
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = self.path[0][i] * self.resolution_map + self.origin_map.position.x
                pose_stamped.pose.position.y = self.path[1][i] * self.resolution_map + self.origin_map.position.y
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.x = 0.0
                pose_stamped.pose.orientation.y = 0.0
                pose_stamped.pose.orientation.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)
            if self.debug_planner_diagnostics and len(path_msg.poses) > 0:
                sample_world = []
                for i, pose in enumerate(path_msg.poses):
                    if i < 5 or i >= max(0, len(path_msg.poses) - 5):
                        sample_world.append(
                            (
                                round(float(pose.pose.position.x), 3),
                                round(float(pose.pose.position.y), 3),
                            )
                        )
                self.get_logger().info(
                    "[PLAN_DIAG] sample_world_points=%s" % str(sample_world)
                )
            path_msg.header.frame_id = self.map_frame
            self.path_pub.publish(path_msg)
            return True
        else:
            self.publish_goal_feedback("goal_invalid")
            self.stop_and_clear_navigation_state("Goal is not valid; stopping and clearing goal.")
            return False
    
    def controller_func(self):
        if not self.startup_spin_done:
            if not self.startup_spin_announced:
                self.startup_spin_announced = True
                self.get_logger().info("Performing one-time 360 startup spin before exploration.")
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = self.startup_spin_angular_speed
            self.vel_pub.publish(self.robot_vel)
            if self.startup_spin_done and not self.startup_spin_completed_announced:
                self.startup_spin_completed_announced = True
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                self.vel_pub.publish(self.robot_vel)
                self.get_logger().info("Startup spin completed. Starting exploration control.")
            return

        if self.startup_spin_done and not self.startup_spin_completed_announced:
            self.startup_spin_completed_announced = True
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
            self.vel_pub.publish(self.robot_vel)
            self.get_logger().info("Startup spin completed. Starting exploration control.")

        if self.path is None and self.goal_pose is not None:
            self.planning()

        if self.path is None or self.goal_pose is None:
            return
        else:
            if len(self.path[0]) == 0:
                self.stop_and_clear_navigation_state("Empty path; stopping.")
                return
            if self.path_index >= len(self.path[0]):
                self.path_index = len(self.path[0]) - 1

            k_v = 0.5
            x, y = self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y
            # self.get_logger().info('Look forward to next path point: "%s" "%s"' % (self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x, self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y))
            # Always follow the planned path points (never switch directly to raw goal pose).
            current_x = self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x
            current_y = self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y
            if (
                self.path_index < len(self.path[0]) - 1
                and np.linalg.norm([current_x - x, current_y - y]) < self.look_forward
            ):
                self.path_index += 1
                current_x = self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x
                current_y = self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y
            x_g, y_g = current_x, current_y
            robot_orientation = self.robot_pose.pose.pose.orientation
            theta = quaternion_to_yaw(robot_orientation)
            inc_x = x_g - x
            inc_y = y_g - y
            if np.linalg.norm([inc_x, inc_y]) < 0.01:
                self.goal_pose = None
                self.path_index = 0
                self.path = None
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                self.vel_pub.publish(self.robot_vel)
                self.get_logger().info('Goal reached!')
                return
            
            dist = np.sqrt(inc_x**2 + inc_y**2)
            desired_yaw = math.atan2(inc_y, inc_x)
            angle_diff = desired_yaw - theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            w = min(self.kw * angle_diff, self.max_angular_velocity)

            # Only move forward if we are roughly facing the right way
            if abs(angle_diff) <= 0.3: # 0.3 before
                v = min(self.kv * dist, self.max_linear_velocity)
            else:
                v = 0.0 # Pivot in place

            if self.mode == "real":
                v = self.apply_deadband_compensation(
                    v,
                    self.linear_deadband,
                    self.linear_zero_epsilon,
                    self.max_linear_velocity,
                )
                w = self.apply_deadband_compensation(
                    w,
                    self.angular_deadband,
                    self.angular_zero_epsilon,
                    self.max_angular_velocity,
                )

            self.robot_vel.linear.x = float(v)
            self.robot_vel.angular.z = float(w)

            self.get_logger().info('Publishing velocity command: linear.x="%s", angular.z="%s"' % (self.robot_vel.linear.x, self.robot_vel.angular.z))

            self.vel_pub.publish(self.robot_vel)

    def is_free_path(self):
        if self.path is None:
            return True
        for i in range(len(self.path[0]) - 1):
            point_1 = np.array([self.path[0][i], self.path[1][i]])
            point_2 = np.array([self.path[0][i+1], self.path[1][i+1]])
            if self.is_free_segment(point_1, point_2, depth=0, max_depth=10) is False:
                return False
        point_1 = np.array([self.path[0][-2], self.path[1][-2]])
        point_2 = np.array([self.path[0][-1], self.path[1][-1]])
        if self.is_free_segment(point_1, point_2, depth=0, max_depth=10) is False:
            return False
        return True
    
    def is_free_segment(self, point_1, point_2, depth, max_depth):
        if depth >= max_depth:
            return True

        if self.point_collided(point_1):
            return False
        
        if self.point_collided(point_2):
            return False
        
        q_mid = (point_1 + point_2) / 2

        if self.point_collided(q_mid):
            return False

        left_segment_free = self.is_free_segment(point_1, q_mid, depth + 1, max_depth)
        if left_segment_free == False:
            return False
        right_segment_free = self.is_free_segment(q_mid, point_2, depth + 1, max_depth)
        return right_segment_free
    
    def point_collided(self, point):
        x = round(point[0])
        y = round(point[1])
        if self.use_transposed_grid:
            grid_map = self.grid_map.T #ELCHIN
        else:
            grid_map = self.grid_map
        return grid_map[x][y] == 1 or grid_map[x+1][y] == 1 or grid_map[x+1][y+1] == 1 or grid_map[x][y+1] == 1 # round up error compensation

    def replanning_func(self):
        if not self.startup_spin_done:
            return
        if self.is_free_path() is True:
            return
        else:
            # Keep previous path/control as fallback in case replanning fails.
            prev_path = self.path
            prev_path_index = self.path_index
            prev_goal_pose = self.goal_pose
            prev_linear = self.robot_vel.linear.x
            prev_angular = self.robot_vel.angular.z

            self.path = None
            self.path_index = 0
            # Clear stale RViz path immediately when invalidating current path.
            empty_path = Path()
            empty_path.header.frame_id = self.map_frame
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty_path)

            replanned = self.planning()
            if not replanned:
                # Restore previous navigation state to avoid dead-stop lockups
                # when a single replanning attempt fails.
                self.path = prev_path
                self.path_index = prev_path_index
                self.goal_pose = prev_goal_pose
                self.robot_vel.linear.x = prev_linear
                self.robot_vel.angular.z = prev_angular
                self.vel_pub.publish(self.robot_vel)

                



def main(args=None):
    rclpy.init(args=args)

    online_motion_planning = OnlineMotionPlanning()

    rclpy.spin(online_motion_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    online_motion_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
