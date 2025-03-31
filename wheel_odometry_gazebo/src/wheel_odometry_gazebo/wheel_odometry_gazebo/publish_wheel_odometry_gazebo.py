#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from control_msgs.msg import DynamicJointState
import math
from rclpy.wait_for_message import wait_for_message
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryNode(Node):
    def __init__(self):
        super().__init__('publish_wheel_odometry_gazebo_node')

        self.WHEEL_RADIUS = 0.165
        self.WHEELBASE = 0.650

        self.wheel_position_left = 0.0
        self.wheel_position_right = 0.0
        self.steering_position = 0.0
        self.prev_wheel_position_left = 0.0
        self.prev_wheel_position_right = 0.0
        self.prev_time = None

        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.get_logger().info("Waiting for /dynamic_joint_states topic to become available...")
        start_time = self.get_clock().now()
        timeout = Duration(seconds=10)

        while rclpy.ok():
            topic_names_and_types = self.get_topic_names_and_types()
            if '/dynamic_joint_states' in dict(topic_names_and_types):
                self.get_logger().info("/dynamic_joint_states topic is available!")
                break

            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error("Timeout waiting for /dynamic_joint_states topic!")
                raise RuntimeError("Timeout waiting for /dynamic_joint_states topic!")

            self.get_logger().info("Topic not available yet, retrying...")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.first_callback = True
        self.dynamic_joints_sub = self.create_subscription(DynamicJointState, '/dynamic_joint_states', self.dynamic_joint_states_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom_gazebo', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update_odometry)

    def dynamic_joint_states_callback(self, msg):
        """
        Callback function to process /dynamic_joint_states messages.
        Extracts wheel position and steering position.
        """
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name == 'rear_left_joint':
                self.wheel_position_left = msg.interface_values[i].values[1]
            if joint_name == 'rear_right_joint':
                self.wheel_position_right = msg.interface_values[i].values[1]
            elif joint_name == 'front_steer_joint':
                self.steering_position = msg.interface_values[i].values[0]

        if self.first_callback == True:
            self.prev_wheel_position_left = self.wheel_position_left
            self.prev_wheel_position_right = self.wheel_position_right
            self.prev_time = self.get_clock().now()
            self.first_callback = False

    def update_odometry(self):


        current_time = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        if self.wheel_position_left == 0.0 and self.wheel_position_right == 0.0 and self.steering_position == 0.0:
            print("waiting for the commands")
            return

        #wheel_noise = random.gauss(0, 0.02)  # Gaussian noise with mean=0, std=0.02
        distance_left = (self.wheel_position_left - self.prev_wheel_position_left) * self.WHEEL_RADIUS
        distance_right = (self.wheel_position_right - self.prev_wheel_position_right) * self.WHEEL_RADIUS

        distance_avg = (distance_left + distance_right) / 2

        self.prev_wheel_position_left = self.wheel_position_left
        self.prev_wheel_position_right = self.wheel_position_right
        steering_angle = self.steering_position

        if abs(steering_angle) > 1e-8:
            turning_radius = self.WHEELBASE / math.tan(steering_angle)
            delta_theta = distance_avg / turning_radius
        else:
            delta_theta = abs(distance_right - distance_left) / self.WHEELBASE

        if abs(steering_angle) > 1e-8:
            delta_x = turning_radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            delta_y = turning_radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        else:
            delta_x = distance_avg * math.cos(self.theta)
            delta_y = distance_avg * math.sin(self.theta)

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation = q

        vel = False
        if vel == True:
            if dt > 0:
                odom_msg.twist.twist.linear.x = distance / dt
                odom_msg.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom_msg)

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
