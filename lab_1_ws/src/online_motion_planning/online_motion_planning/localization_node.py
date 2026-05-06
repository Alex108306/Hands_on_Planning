import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
import numpy as np
from tf2_ros import TransformBroadcaster

from math import atan2, sin, cos

def quaternion_from_euler(roll, pitch, yaw):
    """Helper to replace tf.transformations.quaternion_from_euler"""
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy + sr * sp * cy,
        cr * cp * cy - sr * sp * sy
    ]

def euler_from_quaternion(x, y, z, w):
    """Helper to replace tf.transformations.euler_from_quaternion"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def quaternion_ned_to_enu(q):
    # q is [x, y, z, w] in NED
    q_enu = np.array([q[1], q[0], -q[2], q[3]], dtype=float)
    return q_enu / np.linalg.norm(q_enu)


class DeadReckoning(Node):

    def __init__(self):
        super().__init__('dead_reckoning')
        self.declare_parameter('mode', 'sim')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Initialize frame/profile
        if self.mode == 'real':
            self.world_frame = "odom"
            self.noise_scale = 0.0
        else:
            self.world_frame = "world_enu"
            self.noise_scale = 0.0
        self.base_footprint_frame = "turtlebot/base_footprint"

        # Initialize parameters of the robot
        self.base_length = 0.23
        self.radius_wheel = 0.035
        self.covariance_wheel_encoder = np.diag(np.array([0.01**2, 0.01**2]))
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Pk = np.zeros((3, 3))
        # self.Pk = np.diag([0.001, 0.001, 0.1])
        self.intialize_theta = False

        # Defining node publisher and subcriber
        self.joint_states_sub = self.create_subscription(JointState, "/turtlebot/joint_states", self.joint_state_callback, 20)
        self.imu_sub = self.create_subscription(Imu, "/turtlebot/sensors/imu_data", self.recieve_imu, 20)
        self.odom_pub = self.create_publisher(Odometry, "/turtlebot/odom", 20)
        self.tf_br = TransformBroadcaster(self)

        # Initialize the clock and the last time variable
        self.first_time = True
        self.last_time = self.get_clock().now()

        # Initialize value and flag for IMU update
        self.imu_update_flag = False
        self.imu_orientation = 0.0
        self.imu_covariance = np.zeros((1, 1))

    # Function receive imu infomation data
    def recieve_imu(self, msg):
        # Extract orientation and covariance from IMU message
        orientation_ned = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        orientation_enu = quaternion_ned_to_enu(orientation_ned)
        cov_orientation = msg.orientation_covariance

        # Convert quaternion to Euler angles and extract yaw (theta)
        self.imu_orientation = euler_from_quaternion(orientation_enu[0], orientation_enu[1], orientation_enu[2], orientation_enu[3])[2]  # Adjust for initial orientation
        self.imu_covariance = np.array([[cov_orientation[8]]])  # Assuming covariance for yaw is at index 8
        self.imu_update_flag = True

        if not self.intialize_theta:
            self.theta = self.imu_orientation
            self.intialize_theta = True
    
    # Define motion model
    def f(self, wheel_velocity, dt):

        # Transfer from velocity wheel to robot velocity
        linear_velocity = 1/2 * self.radius_wheel * (wheel_velocity[0, 0] + wheel_velocity[1, 0])
        angular_velocity = (self.radius_wheel/self.base_length) * (-wheel_velocity[0, 0] + wheel_velocity[1, 0])
        
        # Predict the robot pose within timestep dt
        x_k = self.x + cos(self.theta) * linear_velocity * dt
        y_k = self.y + sin(self.theta) * linear_velocity * dt
        theta_k = self.theta + angular_velocity * dt
        if theta_k > np.pi:
            theta_k -= 2 * np.pi
        elif theta_k < -np.pi:
            theta_k += 2 * np.pi
        
        return x_k, y_k, theta_k
    
    # Jacobian of motion model with respect to state
    def Jfx(self, wheel_velocity, dt):
        # Transfer from velocity wheel to robot velocity
        linear_velocity = 1/2 * self.radius_wheel * (wheel_velocity[0, 0] + wheel_velocity[1, 0])

        return np.array([[1, 0, -sin(self.theta) * linear_velocity * dt],
                         [0, 1, cos(self.theta) * linear_velocity * dt],
                         [0, 0, 1]])
    
    # Jacobian of motion model with respect to noise
    def Jfw(self, wheel_velocity, dt):
        return np.array([[1/2 * cos(self.theta) * dt, 1/2 * cos(self.theta) * dt],
                         [1/2 * sin(self.theta) * dt, 1/2 * sin(self.theta) * dt],
                         [- self.radius_wheel/self.base_length * dt, self.radius_wheel/self.base_length * dt]])
    
    # Observation model
    def h(self, xk_bar):
        return np.array([xk_bar[2]])
    
    # Jacobian of observation model with respect to state
    def Hk(self):
        return np.array([[0, 0, 1]])
    
    # Jacobian of observation model with respect to noise
    def Vk(self):
        return np.identity(1)
    
    # Prediction function
    def Prediction(self, wheel_velocity, dt):
        # Predict the position of the robot and the covariance
        x_bar, y_bar, theta_bar = self.f(wheel_velocity, dt)
        Jfx = self.Jfx(wheel_velocity, dt)
        Jfw = self.Jfw(wheel_velocity, dt)
        Pk_bar = Jfx @ self.Pk @ Jfx.T + Jfw @ self.covariance_wheel_encoder @ Jfw.T
        xk_bar = np.array([x_bar, y_bar, theta_bar]).reshape(3,1)

        return xk_bar, Pk_bar

    # Update function
    def Update(self, xk_bar, Pk_bar):

        # Get matrix and value for updating process
        Hk = self.Hk()
        Vk = self.Vk()
        zk = np.array([[self.imu_orientation]])
        Rk = self.imu_covariance

        # Update process
        Kk = Pk_bar @ Hk.T @ np.linalg.inv(Hk @ Pk_bar @ Hk.T + Vk @ Rk @ Vk.T)
        xk = xk_bar + Kk @ (zk - self.h(xk_bar))
        Pk = (np.identity(len(Pk_bar)) - Kk @ Hk) @ Pk_bar @ (np.identity(len(Pk_bar)) - Kk @ Hk).T
        return xk, Pk

    
    def joint_state_callback(self, msg):
        if not self.intialize_theta:
            return
        current_time = self.get_clock().now()

        if self.first_time:
            self.first_time = False
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <=0:
            return
        
        left_wheel_velocity = msg.velocity[0]
        right_wheel_velocity = msg.velocity[1]

        # Adding noise to the wheel encoder sensor
        # wheel_velocity = np.array([[left_wheel_velocity], [right_wheel_velocity]]) + np.random.normal(np.zeros((2,1)), np.array([np.sqrt(self.covariance_wheel_encoder.diagonal())]).T)
        wheel_velocity = (
            np.array([[left_wheel_velocity], [right_wheel_velocity]])
            + self.noise_scale
            * np.random.normal(
                np.zeros((2, 1)),
                np.array([np.sqrt(self.covariance_wheel_encoder.diagonal())]).T,
            )
        )
        # Predict pose of robot with covariance
        xk_bar, Pk_bar = self.Prediction(wheel_velocity, dt)

        # Update function so use predict
        if self.imu_update_flag == True:
            xk, Pk = self.Update(xk_bar, Pk_bar)
            self.x = xk[0, 0]
            self.y = xk[1, 0]
            self.theta = xk[2, 0]
            self.Pk = Pk
            self.imu_update_flag = False
        else:
            self.x = xk_bar[0, 0]
            self.y = xk_bar[1, 0]
            self.theta = xk_bar[2, 0]
            self.Pk = Pk_bar
    
        # Transfer from velocity wheel to robot velocity
        linear_velocity = 1/2 * self.radius_wheel * (wheel_velocity[0, 0] + wheel_velocity[1, 0])
        angular_velocity = (self.radius_wheel/self.base_length) * (-wheel_velocity[0, 0] + wheel_velocity[1, 0])

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.world_frame
        odom_msg.child_frame_id = self.base_footprint_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # Fill 6x6 covariance matrix (flattened)
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.Pk[0:2, 0:2]
        cov[0:2, 5] = self.Pk[0:2, 2]
        cov[5, 0:2] = self.Pk[2, 0:2]
        cov[5, 5] = self.Pk[2, 2]
        odom_msg.pose.covariance = cov.flatten().tolist()

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_footprint_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    dead_reckoning = DeadReckoning()

    rclpy.spin(dead_reckoning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dead_reckoning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
