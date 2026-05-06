import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformListener

import math

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    [https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/]
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class GridMap:
    """
    Probabilistic occupancy grid map. 
    """
    
    # Maximum and minimum cell loggoods (is equivalen to p_max = 0.999, p_min = 0.001)
    LMAX = 6.91 
    LMIN = -6.91

    # Initialize gridmap class
    def __init__(self, center, cell_size=0.1, map_size=10):
        self.cell_size = cell_size
        self.grid = np.zeros((int(map_size/cell_size), int(map_size/cell_size))) + 0.5 # Initialize grid with 0.5 probability
        self.origin = np.array(center) - np.array([map_size, map_size])/2

    # return map
    def get_map(self):
        return self.grid

    # return origin
    def get_origin(self):
        return self.origin
    
    # Transform position wrt frame origin to cell indexes
    def __position_to_cell__(self, position):
        origin = self.get_origin()
        cell_index = np.floor((position - origin) / self.cell_size)
        cell_index = cell_index.astype(int)
        return cell_index

    def bresenham(self, x0, y0, x1, y1):
        """Bresenham's line algorithm."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points

    # Update cell uv with probability p
    def __update_cell__(self, uv, p):
        # Check that probability p is between 0 and 1 but not 0 or 1
        if p > 0.0 and p < 1.0:
            # Compute inverse sensor model
            inverse_sensor_model = np.log(p / (1 - p))
            # Update cell and saturate between GridMap.LMIN and GridMap.LMAX 
            recursive_term = np.log(self.grid[uv[0], uv[1]]/(1 - self.grid[uv[0], uv[1]]))
            log_cell = np.clip(inverse_sensor_model + recursive_term, a_min=GridMap.LMIN, a_max=GridMap.LMAX)
            self.grid[uv[0], uv[1]] = np.exp(log_cell) / (1 + np.exp(log_cell))

    # Update all cells traversed by a ray
    def add_ray(self, ray_init_position, ray_angle, ray_range, p):
        # Compute ray final position using ray start, angle and range
        ray_final_position = np.array([ray_init_position[0] + np.cos(ray_angle) * ray_range, ray_init_position[1] + np.sin(ray_angle) * ray_range])

        # Transform intial and final position to cell indexes
        ray_init_index = self.__position_to_cell__(ray_init_position)
        ray_final_index = self.__position_to_cell__(ray_final_position)

        # Use Bresenham's algorithm to get traversed cells
        list_traversed_cells = list(self.bresenham(ray_init_index[0], ray_init_index[1], ray_final_index[0], ray_final_index[1]))

        # Update as no occupied (probability 1-p) the cells between start and end - 1 
        # and the last cell with probability p
        for i in range(len(list_traversed_cells) - 1):
            self.__update_cell__(list_traversed_cells[i], 1 - p) # Update the intermediate cells with probability 1 - p
        
        self.__update_cell__(list_traversed_cells[-1], p) # Update the last cell with probability p

class GridMapping(Node):

    def __init__(self):
        super().__init__('grid_mapping')
        self.declare_parameter('mode', 'sim')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.world_frame = "odom" if self.mode == "real" else "world_ned"
        self.map_publish_period = 0.7 if self.mode == "real" else 0.5

        # Subscribers and publishers
        self.scan_sub = self.create_subscription(LaserScan, '/turtlebot/scan', self.recieve_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/turtlebot/odom', self.recieve_odom, 10)
        self.grid_map_pub = self.create_publisher(OccupancyGrid, '/projected_map', 10)

        # Creating timer
        self.timer_publish = self.create_timer(self.map_publish_period, self.process_grid_map)

        # TF
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Robot state and grid map
        self.robot_x = None
        self.robot_y = None
        self.robot_theta = None
        self.grid_map = None
        # self.grid_map = GridMap(center=(self.robot_x, self.robot_y), cell_size=0.02, map_size=20)

        # World frame and robot frame
        self.odom_frame = None
        self.laser_frame = None

        # Initial scan message
        self.scan_msg = None
        
    
    def recieve_odom(self, msg):
        self.odom_frame = msg.header.frame_id
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        robot_orientation = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion(robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w)
        if self.grid_map is None:
            self.grid_map = GridMap(center=(self.robot_x, self.robot_y), cell_size=0.02, map_size=20)
            self.get_logger().info(f'{self.robot_x}, {self.robot_y}')

    def recieve_scan(self, msg):
        self.scan_msg = msg
        
    
    def process_grid_map(self):
        if self.grid_map is None:
            return
        if self.scan_msg is None:
            return
        self.laser_frame = self.scan_msg.header.frame_id
        scan_time = rclpy.time.Time.from_msg(self.scan_msg.header.stamp)
        ranges_list = self.scan_msg.ranges
        # self.get_logger().info('Received laser scan with {} ranges'.format(len(ranges_list)))
        if self.odom_frame != None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame, self.laser_frame, scan_time,
                    timeout=rclpy.duration.Duration(seconds=2))
            except Exception as e:
                self.get_logger().error('Transform lookup failed: {}'.format(e))
                return
            
            # self.get_logger().info('Transform lookup successful: {}'.format(transform))
            laser_base_x, laser_base_y = transform.transform.translation.x, transform.transform.translation.y
            laser_angle = transform.transform.rotation
            _, _, laser_theta = euler_from_quaternion(laser_angle.x, laser_angle.y, laser_angle.z, laser_angle.w)


            for i in range(len(ranges_list)):
                if ranges_list[i] < self.scan_msg.range_max and ranges_list[i] > self.scan_msg.range_min:
                    ray_angle = self.scan_msg.angle_min + i * self.scan_msg.angle_increment + laser_theta
                    ray_range = ranges_list[i]
                    self.grid_map.add_ray([laser_base_x, laser_base_y], ray_angle, ray_range, 0.9)

            self.publish_grid_map()

    def publish_grid_map(self):
        grid_map_msg = OccupancyGrid()
        grid_map_msg.header.frame_id = self.world_frame
        grid_map_msg.info.resolution = self.grid_map.cell_size
        grid_map_msg.info.width = self.grid_map.grid.shape[0]
        grid_map_msg.info.height = self.grid_map.grid.shape[1]
        grid_map_msg.info.origin.position.x = self.grid_map.origin[0]
        grid_map_msg.info.origin.position.y = self.grid_map.origin[1]
        grid_map_msg.info.origin.position.z = 0.0
        grid_map_msg.info.origin.orientation.x = 0.0
        grid_map_msg.info.origin.orientation.y = 0.0
        grid_map_msg.info.origin.orientation.z = 0.0
        grid_map_msg.info.origin.orientation.w = 1.0
        grid_map_msg.data = (self.grid_map.grid.T.flatten() * 100).astype(int).tolist()
        self.grid_map_pub.publish(grid_map_msg)

        # self.get_logger().info('Published grid map with origin ({}, {}) and resolution {}'.format(self.grid_map.origin[0], self.grid_map.origin[1], self.grid_map.cell_size))

def main(args=None):
    rclpy.init(args=args)

    grid_mapping = GridMapping()

    # executor = rclpy.executors.MultiThreadedExecutor()

    rclpy.spin(grid_mapping)

    grid_mapping.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

