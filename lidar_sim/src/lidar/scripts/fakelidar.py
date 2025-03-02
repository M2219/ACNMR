#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class FakeLidar(Node):
    def __init__(self):
        super().__init__("fake_lidar")

        # LiDAR parameters
        self.lidar_range = 50.0  # Max scan range in meters
        self.angle_min = -1.57  # -15 degrees
        self.angle_max = 1.57   # 15 degrees
        self.angle_increment = 0.01  # Resolution (radians per step) = 0.36 deg

        # Robot position
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS Profile for Map Service
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # Subscribe to odometry and wait for the first valid message
        self.get_logger().info("Waiting for /odom...")
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.wait_for_first_odom()

        # Wait for the static map
        self.get_logger().info("Waiting for /map_server/load_map service...")
        self.map_client = self.create_client(GetMap, "/map_server/map")

        while not self.map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Still waiting for /map_server/map service...")

        self.get_logger().info("Requesting map from /map_server/_map...")
        self.map_request()
        # Publisher for fake LiDAR scan
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        self.get_logger().info("Fake LiDAR node started!")
        self.timer = self.create_timer(0.05, self.publish_scan)  # 20 Hz

    def wait_for_first_odom(self):
        """Wait until the first odometry message is received"""
        while self.robot_x is None and rclpy.ok():
            rclpy.spin_once(self)

    def odom_callback(self, msg):
        """ Update the robot position from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        self.robot_yaw = quat2euler([quat.w, quat.x, quat.y, quat.z])[2]

    def map_request(self):
        """ Request the map from the map server """
        req = GetMap.Request()
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.map_data = future.result().map
            self.get_logger().info("Received /map!")
        else:
            self.get_logger().error("Failed to receive /map!")

    def simulate_lidar_scan(self):
        """ Simulate a LiDAR scan using the static map """
        if self.robot_x is None or self.robot_y is None:
            self.get_logger().warn("Odometry not yet received, skipping LiDAR scan...")
            return []

        scan_data = []
        angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        for angle in angles:
            scan_range = self.lidar_range  # Start with max range
            ray_x, ray_y = self.robot_x, self.robot_y

            # Step along the ray to check for obstacles
            for _ in range(int(self.lidar_range * 10)):  # 20 steps per meter
                ray_x += np.cos(self.robot_yaw + angle) * 0.05  # Step size
                ray_y += np.sin(self.robot_yaw + angle) * 0.05

                # Convert to map indices
                map_x = int((ray_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                map_y = int((ray_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

                if 0 <= map_x < self.map_data.info.width and 0 <= map_y < self.map_data.info.height:
                    if self.map_data.data[map_y * self.map_data.info.width + map_x] > 50:  # Occupied space
                        scan_range = np.sqrt((ray_x - self.robot_x)**2 + (ray_y - self.robot_y)**2)
                        break  # Stop at first obstacle

            scan_data.append(scan_range)

        return scan_data

    def publish_scan(self):
        """ Publish the fake laser scan """
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.lidar_range
        scan_msg.ranges = self.simulate_lidar_scan()
        scan_msg.intensities = [1.0 - (r / self.lidar_range) if r < self.lidar_range else 0.0 for r in scan_msg.ranges]

        self.scan_pub.publish(scan_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
