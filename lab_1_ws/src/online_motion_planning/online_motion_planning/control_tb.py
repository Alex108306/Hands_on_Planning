import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from math import atan2

def quaternion_to_yaw(q):
    """Extract yaw (theta) from geometry_msgs quaternion (x, y, z, w)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


class LowLevelMotionControl(Node):

    def __init__(self):
        super().__init__('low_level_motion_control')
        self.odom_sub = self.create_subscription(Odometry, '/turtlebot/odom', self.recieve_odom, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.recieve_goal_pose, 10)
        # self.goal_pose_service = self.create_service(PoseStamped, '/goto', self.goto_calling)
        self.vel_pub = self.create_publisher(Twist, '/turtlebot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.controller_func)
        self.robot_vel = Twist()
        self.robot_pose = PoseStamped()
        self.goal_pose = None
    
    def recieve_odom(self, msg):
        self.robot_pose.pose = msg.pose.pose
        # self.get_logger().info('Received robot pose: "%s"' % self.robot_pose)
    
    def recieve_goal_pose(self, msg):
        self.goal_pose = PoseStamped()
        self.goal_pose = msg.pose
        # self.get_logger().info('Received goal pose: "%s"' % self.goal_pose)

    def goto_calling(self, request, response):
        self.goal_pose = request.pose
    
    def controller_func(self):
        if self.goal_pose == None:
            pass
        else:
            
            k_w = 0.5
            k_v = 0.5
            x_g, y_g = self.goal_pose.position.x, self.goal_pose.position.y
            x, y = self.robot_pose.pose.position.x, self.robot_pose.pose.position.y
            robot_orientation = self.robot_pose.pose.orientation
            theta = quaternion_to_yaw(robot_orientation)
            inc_x = x_g - x
            inc_y = y_g - y
            # self.get_logger().info('Error: "%s" "%s"' % (inc_x, inc_y))
            w = k_w * (atan2(inc_y, inc_x) - theta)
            # self.get_logger().info('Angular: "%s"' % abs(w))
            if abs(w) < 0.1:
                v = k_v * ((inc_x ** 2 + inc_y ** 2) ** 0.5)
            else: 
                v = 0.0
            self.robot_vel.linear.x = v
            self.robot_vel.angular.z = w
            self.vel_pub.publish(self.robot_vel)
            if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
                self.goal_pose = None
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
                self.vel_pub.publish(self.robot_vel)
                self.get_logger().info('Goal reached!')



def main(args=None):
    rclpy.init(args=args)

    low_level_motion_control = LowLevelMotionControl()

    rclpy.spin(low_level_motion_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    low_level_motion_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()