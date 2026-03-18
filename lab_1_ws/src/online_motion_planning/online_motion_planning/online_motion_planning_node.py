import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster
from online_motion_planning.RRT_star_algorithm import RRT_Star, fill_path, convert_to_path_index
from online_motion_planning.Point import Point

from PIL import Image

from math import atan2

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

        # Defining node publisher and subcriber
        self.odom_sub = self.create_subscription(Odometry, "/turtlebot/odom", self.odom_callback, 10)
        self.grid_map_sub = self.create_subscription(OccupancyGrid, "/projected_map", self.grid_map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)
        self.vel_pub = self.create_publisher(Twist, '/turtlebot/cmd_vel', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, "/inflated_map", 10)
        self.create_timer(1.0, self.replanning_func)
        self.create_timer(0.1, self.controller_func)

        # Initialize map variable
        self.grid_map = None
        self.resolution_map = None
        self.width_map, self.height_map = None, None
        self.origin_map = None

        # Initialize robot radius
        self.robot_radius = 0.115

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

        # Initizalize control motion variable
        self.robot_vel = Twist()
        self.look_forward = 0.1

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

    def odom_callback(self, msg):
        self.robot_pose = msg

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
        inflated_map_msg.header.frame_id = "odom" # world_ned for simulation
        inflated_map_msg.info.resolution = self.resolution_map
        inflated_map_msg.info.width = self.width_map
        inflated_map_msg.info.height = self.height_map
        inflated_map_msg.info.origin = self.origin_map
        inflated_map_msg.data = (self.grid_map.flatten() * 100).astype(int).tolist()
        self.inflated_map_pub.publish(inflated_map_msg)


    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        if self.grid_map is None or self.robot_pose is None:
            self.get_logger().info('Waiting for grid map and robot pose to be available...')
            return
        self.planning()
        
    def planning(self):
        # self.get_logger().info('Planning path to goal: "%s" "%s"' % (self.goal_pose.position.x, self.goal_pose.position.y))
        if self.is_valid(self.goal_pose.position.x, self.goal_pose.position.y):   
            path_msg = Path()
            robot_pose = self.to_index(self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y)
            goal_pose = self.to_index(self.goal_pose.position.x, self.goal_pose.position.y)
            start = Point(robot_pose[0], robot_pose[1])
            goal = Point(goal_pose[0], goal_pose[1])
            self.grid_map = self.grid_map.astype(int)
            rrt_star = RRT_Star(grid_map=self.grid_map.T, K=self.K, delta_q=self.delta_q, p=self.p, max_distance=self.max_range, q_start=start, q_goal=goal)
            path_first_found, path_converge, edge_goal = rrt_star.run_RRT_Star()
            if len(edge_goal) == 0:
                self.get_logger().info("No path found")
                return
            _, _, path = fill_path(path_converge[0], path_converge[1], edge_goal)
            path_smoothed = rrt_star.smoothing_function(path, path_converge[0])
            self.path = convert_to_path_index(path_converge[0], path_converge[1], path_smoothed)
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
            path_msg.header.frame_id = "odom" # world_enu for simulation
            self.path_pub.publish(path_msg)
        else:
            self.get_logger().info("Goal is not valid")
    
    def controller_func(self):
        if self.path is None or self.goal_pose is None:
            return
        else:
            k_v = 0.5
            x, y = self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y
            # self.get_logger().info('Look forward to next path point: "%s" "%s"' % (self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x, self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y))
            if self.path_index == len(self.path[0]) - 1:
                x_g, y_g = self.goal_pose.position.x, self.goal_pose.position.y
            else:
                if np.linalg.norm([self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x - x,
                                   self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y - y]) < self.look_forward:
                    self.path_index += 1
                x_g, y_g = self.path[0][self.path_index] * self.resolution_map + self.origin_map.position.x, self.path[1][self.path_index] * self.resolution_map + self.origin_map.position.y
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
            w = wrap_to_pi(atan2(inc_y, inc_x) - theta)
            # self.get_logger().info(f'{w}')
            if abs(w) > 0.1:  # e.g., threshold = 0.3
                v = 0.0             # stop moving forward, rotate in place
            else:
                v = min(k_v * ((inc_x ** 2 + inc_y ** 2) ** 0.5), 0.5)

            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w
            # self.get_logger().info(f"{v}, {w}")
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
        grid_map = self.grid_map.T
        return grid_map[x][y] == 1 or grid_map[x+1][y] == 1 or grid_map[x+1][y+1] == 1 or grid_map[x][y+1] == 1 # round up error compensation

    def replanning_func(self):
        if self.is_free_path() is True:
            return
        else:
            self.path = None
            self.path_index = 0
            self.robot_vel.linear.x = 0.0
            self.robot_vel.angular.z = 0.0
            self.vel_pub.publish(self.robot_vel)
            self.planning()

                



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
