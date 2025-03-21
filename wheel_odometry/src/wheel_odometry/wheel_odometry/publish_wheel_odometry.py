#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Int32  # Assuming pulse counts are published as Int32
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('publish_wheel_odometry_node')

        # Constants
        self.WHEEL_RADIUS = 0.1  # meters
        self.ENCODER_RESOLUTION = 1000  # pulses per revolution (for rear wheels)
        self.STEERING_RESOLUTION = 500  # pulses per revolution (for steering motor)
        self.WHEELBASE = 0.5  # meters (distance between front and rear axles)

        # Initial pose
        self.x, self.y, self.theta = 0.0, 0.0, 0.0  # Initial position and orientation

        # Initialize pulse counts
        self.pulse_left = 0
        self.pulse_right = 0
        self.pulse_steering = 0

        # ROS 2 Publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)

        # ROS 2 Subscribers for pulse counts
        self.left_pulse_sub = self.create_subscription(
            Int32, '/hunter_status/motor_rear_left_pulse', self.left_pulse_callback, 10)
        self.right_pulse_sub = self.create_subscription(
            Int32, '/hunter_status/motor_rear_right_pulse', self.right_pulse_callback, 10)
        self.steer_pulse_sub = self.create_subscription(
            Int32, '/hunter_status/motor_steer_pulse', self.steer_pulse_callback, 10)

        # ROS 2 Timer to periodically update and publish odometry
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10 Hz

    def left_pulse_callback(self, msg):
        """Callback for left rear wheel pulse count."""
        self.pulse_left = msg.data

    def right_pulse_callback(self, msg):
        """Callback for right rear wheel pulse count."""
        self.pulse_right = msg.data

    def steer_pulse_callback(self, msg):
        """Callback for steering motor pulse count."""
        self.pulse_steering = msg.data

    def update_odometry(self):
        # Step 1: Calculate the steering angle from the steering motor's pulse count
        steering_angle = self.calculate_steering_angle(self.pulse_steering)

        # Step 2: Calculate distance traveled by each rear wheel
        distance_left = (self.pulse_left / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS)
        distance_right = (self.pulse_right / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS)

        # Step 3: Calculate average distance
        distance_avg = (distance_left + distance_right) / 2

        # Step 4: Calculate change in orientation (yaw)
        delta_theta = (distance_right - distance_left) / self.WHEELBASE

        # Step 5: Calculate change in position (x, y)
        delta_x = distance_avg * math.cos(self.theta + steering_angle)
        delta_y = distance_avg * math.sin(self.theta + steering_angle)

        # Step 6: Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Step 7: Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to a quaternion for orientation
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q

        # Set velocity (optional, can be computed if needed)
        #odom_msg.twist.twist.linear.x = distance_avg / self.timer.timer_period_ns * 1e9  # Linear velocity in m/s
        #odom_msg.twist.twist.angular.z = delta_theta / self.timer.timer_period_ns * 1e9  # Angular velocity in rad/s

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f"Published Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.2f}°")

    def calculate_steering_angle(self, pulse_steering):
        """
        Calculate the steering angle from the steering motor's pulse count.
        Assumes the steering motor's encoder resolution is known.
        """
        # Convert pulse count to steering angle in radians
        steering_angle = (pulse_steering / self.STEERING_RESOLUTION) * (2 * math.pi)
        return steering_angle

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.
        """
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
