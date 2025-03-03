#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class RealtimePlotter(Node):
    def __init__(self):
        super().__init__('realtime_plotter')

        self.odom_received = False
        self.target_received = False
        self.steering_received = False
        self.target_yaw_received = False
        self.target_error_received = False
        self.target_time_received = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/target_x', self.target_x_callback, 10)
        self.create_subscription(Float64, '/target_y', self.target_y_callback, 10)
        self.create_subscription(Float64, '/steering', self.steering_callback, 10)
        self.create_subscription(Float64, '/target_yaw', self.target_yaw_callback, 10)
        self.create_subscription(Float64, '/target_error', self.target_error_callback, 10)
        self.create_subscription(Float64, '/target_time', self.target_time_callback, 10)

        self.get_logger().info("Waiting for messages on /odom, /target_x, /target_y, /steering, /target_yaw, /target_error, and /target_time...")

        self.x_data, self.y_data = [], []
        self.target_x, self.target_y = None, None
        self.target_time_data, self.steering_data, self.target_yaw_data, self.target_error_data = [], [], [], []

        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 12))
    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info("Received first /odom message!")
            self.odom_received = True
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)
        self.update_plot()

    def target_x_callback(self, msg):
        self.target_x = msg.data
        self.check_target_received()

    def target_y_callback(self, msg):
        self.target_y = msg.data
        self.check_target_received()

    def target_yaw_callback(self, msg):
        if not self.target_yaw_received:
            self.get_logger().info("Received first /target_yaw message!")
            self.target_yaw_received = True
        self.target_yaw_data.append(msg.data)
        self.update_plot()

    def steering_callback(self, msg):
        if not self.steering_received:
            self.get_logger().info("Received first /steering message!")
            self.steering_received = True
        self.steering_data.append(msg.data)
        self.update_plot()

    def target_error_callback(self, msg):
        if not self.target_error_received:
            self.get_logger().info("Received first /target_error message!")
            self.target_error_received = True
        self.target_error_data.append(msg.data)
        self.update_plot()

    def check_target_received(self):
        if self.target_x is not None and self.target_y is not None and not self.target_received:
            self.get_logger().info(f"Received first /target_x and /target_y: ({self.target_x}, {self.target_y})")
            self.target_received = True
        self.update_plot()

    def target_time_callback(self, msg):
        self.target_time_data.append(msg.data)
        self.target_time_received = True
        self.update_plot()

    def update_plot(self):
        if self.x_data and self.y_data:
            self.axs[0].clear()
            self.axs[0].plot(self.x_data, self.y_data, 'bo-', label="Odometry Path")
            if self.target_x is not None and self.target_y is not None:
                self.axs[0].plot(self.target_x, self.target_y, 'ro', markersize=10, label="Target Position")
            self.axs[0].set_xlabel("X Position")
            self.axs[0].set_ylabel("Y Position")
            self.axs[0].set_title("Robot Path with Target Position")
            self.axs[0].legend()
            self.axs[0].grid()

        if self.target_time_received and len(self.target_time_data) == len(self.steering_data):
            self.axs[1].clear()
            self.axs[1].plot(self.target_time_data, self.steering_data, 'r-', label="Steering Angle")
            if len(self.target_yaw_data) == len(self.target_time_data):
                self.axs[1].plot(self.target_time_data, self.target_yaw_data, 'b-', label="Target Yaw")
            self.axs[1].set_xlabel("Target Time (s)")
            self.axs[1].set_ylabel("Angle")
            self.axs[1].set_title("Steering and Target Yaw vs Target Time")
            self.axs[1].legend()
            self.axs[1].grid()

        if self.target_time_received and len(self.target_time_data) == len(self.target_error_data):
            self.axs[2].clear()
            self.axs[2].plot(self.target_time_data, self.target_error_data, 'g-', label="Target Error")
            self.axs[2].set_xlabel("Target Time (s)")
            self.axs[2].set_ylabel("Error")
            self.axs[2].set_title("Target Error vs Target Time")
            self.axs[2].legend()
            self.axs[2].grid()

        plt.draw()
        plt.pause(0.001)
def main(args=None):
    rclpy.init(args=args)
    node = RealtimePlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
