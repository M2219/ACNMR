#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from hunter_msgs.msg import HunterStatus
from rclpy.wait_for_message import wait_for_message
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import random

class OdometryNode(Node):
    def __init__(self):
        super().__init__('publish_wheel_odometry_node')
        # 0.15485
        # 0.15186
        self.WHEEL_RADIUS_LEFT = 0.15485
        self.WHEEL_RADIUS_RIGHT = 0.15186
        self.ENCODER_RESOLUTION = 409500  # pulses per revolution (for rear wheels)
        self.STEERING_RESOLUTION = 409500  # pulses per revolution (for steering motor)
        self.WHEELBASE = 0.645  # meters (distance between front and rear axles)

        self.wheel_calibration = False
        left_ticks = []
        right_ticks = []
        self.wheel_calibration_factor = 1.0

        self.en_range = 16777215

        # Initial pose
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.pulse_left = 0
        self.pulse_right = 0

        self.steering_angle = 0
        self.prev_pulse_steering = 0

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Waiting for the first message on /hunter_status...")
        try:
            wait_for_message(
                HunterStatus,
                self,
                '/hunter_status',
            )
            self.get_logger().info("First message received on /hunter_status!")
        except Exception as e:
            self.get_logger().error(f"Failed to receive a message on /hunter_status: {e}")
            raise RuntimeError("Failed to receive a message on /hunter_status!")

        self.first_callback = True
        self.subscription = self.create_subscription(HunterStatus, '/hunter_status', self.hunter_status_callback, 10)
        self.timer = self.create_timer(0.02, self.update_odometry)

    def hunter_status_callback(self, msg):
        """
        Callback function to process /hunter_status messages.
        Extracts pulse counts for steering, right wheel, and left wheel.
        """
        self.pulse_steering = msg.actuator_states[0].pulse_count
        self.pulse_right = msg.actuator_states[1].pulse_count
        self.pulse_left = msg.actuator_states[2].pulse_count
        self.steering_angle = msg.steering_angle
        self.linear_velocity = msg.linear_velocity

        if self.first_callback:
            self.prev_pulse_steering = self.pulse_steering
            self.prev_pulse_left = self.pulse_left
            self.prev_pulse_right = self.pulse_right
            self.first_callback = False

        # Log the extracted pulse counts
        #self.get_logger().info(
        #    f"Steering Pulse: {self.pulse_steering}, "
        #    f"Right Wheel Pulse: {self.pulse_right}, "
        #    f"Left Wheel Pulse: {self.pulse_left}"
        #)

    def pulse_to_steering_angle(self, pulse_count, steering_direction):
        """
        Convert pulse count to steering angle using piecewise linear calibration.
        """
        m_left = 4.105775425733488e-06
        m_right = 3.4601135362025746e-08

        if steering_direction == "left":
            steering_angle_deg = m_left * pulse_count
        elif steering_direction == "right":
            steering_angle_deg = m_right * pulse_count
        else:
            steering_angle_deg = 0.0

        return steering_angle_deg

    def calibrate_wheels(self):
        """Calibrates the wheel radius using encoder readings over a set number of revolutions."""
        print("Starting wheel calibration...")

        # Command straight motion at low speed

        self.left_ticks.append(self.pulse_left)
        self.right_ticks.append(self.pulse_right)

        # Calculate effective wheel distances
        left_dist = ((max(left_ticks) - min(left_ticks)) / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS)
        right_dist = ((max(right_ticks) - min(right_ticks)) / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS)

        # Compute calibration factor (>1 if left wheel is effectively larger)
        self.wheel_calibration_factor = right_dist / left_dist
        print(f"Calibration factor: {self.wheel_calibration_factor:.4f}")

        # Stop motion after calibration
        self.command_velocity(0, 0)

    def update_odometry(self):

        # Step 1: Calculate delta pulse counts (accounting for rollover)
        delta_pulse_left = self.calculate_delta_pulse(self.pulse_left, self.prev_pulse_left, self.en_range)
        delta_pulse_right = - self.calculate_delta_pulse(self.pulse_right, self.prev_pulse_right, self.en_range)
        #delta_pulse_steering = self.calculate_delta_pulse(self.pulse_steering, self.prev_pulse_steering, self.en_range)
        #print("self.pulse_steering", self.pulse_steering)
        #print("delta_pulse_steering", delta_pulse_steering)

        # Update previous pulse counts
        self.prev_pulse_left = self.pulse_left
        self.prev_pulse_right = self.pulse_right
        #self.prev_pulse_steering = self.pulse_steering

        # Step 2: Calculate distance traveled by each rear wheel
        distance_left = (delta_pulse_left / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS_LEFT)
        distance_right = (delta_pulse_right / self.ENCODER_RESOLUTION) * (2 * math.pi * self.WHEEL_RADIUS_RIGHT * self.wheel_calibration_factor)

        if self.wheel_calibration:
            self.calibrate_wheels()

        # Step 3: Calculate the steering angle from the steering motor's pulse count
        #print("before", delta_pulse_steering)
        #delta_steering_angle = (delta_pulse_steering / self.STEERING_RESOLUTION) * (2 * math.pi)
        #print("after", delta_steering_angle)
        #self.steering_angle += delta_steering_angle  # Update the current steering angle
        #print("self.steering_angle", self.steering_angle)

        #self.steering_angle = self.pulse_to_steering_angle(self.pulse_steering, "left")

        # Normalize the steering angle to the range [-pi, pi]
        #self.steering_angle = (self.steering_angle + math.pi) % (2 * math.pi) - math.pi

        #print("self.pulse_steering,", self.pulse_steering)
        #print("after self.steering_angle", self.steering_angle)

        # Step 4: Calculate average distance
        distance_avg = (distance_left + distance_right) / 2
        #self.steering_angle = self.steering_angle + random.gauss(0, 0.05)
        # Step 5: Calculate change in orientation (yaw)
        if abs(self.steering_angle) > 1e-6:
            turning_radius = self.WHEELBASE / math.tan(self.steering_angle)
            delta_theta = distance_avg / turning_radius
        else:
            delta_theta = (distance_right - distance_left) / self.WHEELBASE

        # Step 6: Calculate change in position (x, y)
        if abs(self.steering_angle) > 1e-6:
            delta_x = turning_radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = turning_radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        else:
            delta_x = distance_avg * math.cos(self.theta)
            delta_y = distance_avg * math.sin(self.theta)

        # Step 7: Update pose
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Step 8: Create and publish Odometry message
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
        # odom_msg.twist.twist.linear.x = self.linear_velocity
        # odom_msg.twist.twist.angular.z = delta_theta / self.timer.timer_period_ns * 1e9  # Angular velocity in rad/s

        self.odom_pub.publish(odom_msg)
        self.publish_tf()
        #self.get_logger().info(f"Published Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={math.degrees(self.theta):.2f}°")

    def calculate_delta_pulse(self, current_pulse, previous_pulse, resolution):
        """
        Calculate the delta pulse count, accounting for rollover.
        """
        delta_pulse = current_pulse - previous_pulse
        if delta_pulse < -resolution / 2:
            delta_pulse += resolution
        elif delta_pulse > resolution / 2:
            delta_pulse -= resolution
        return delta_pulse

    def publish_tf(self):
        """
        Publish the TF transformation between odom and base_link.
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        tf_msg.transform.rotation = q

        self.tf_broadcaster.sendTransform(tf_msg)

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
