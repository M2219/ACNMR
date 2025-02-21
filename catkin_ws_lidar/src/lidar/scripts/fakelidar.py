#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class FakeLidar:
    def __init__(self):
        rospy.init_node("fake_lidar")

        # LiDAR parameters
        self.lidar_range = 5.0  # Max scan range in meters
        self.angle_min = -1.57  # -90 degrees
        self.angle_max = 1.57   # 90 degrees
        self.angle_increment = 0.01  # Resolution (radians per step)

        # Robot position
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        # Subscribe to odometry and wait for the first valid message
        rospy.loginfo("Waiting for /odom...")
        odom_msg = rospy.wait_for_message("/odom", Odometry)
        self.odom_callback(odom_msg)
        rospy.loginfo("Received /odom!")

        # Wait for the static map
        rospy.loginfo("Waiting for /map service...")
        rospy.wait_for_service('/static_map')
        self.map_service = rospy.ServiceProxy('/static_map', GetMap)
        self.map_data = self.map_service().map
        rospy.loginfo("Received /map!")

        # Publisher for fake LiDAR scan
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)

        # Subscribe to odometry for continuous updates
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.loginfo("Fake LiDAR node started!")
        self.run()

    def odom_callback(self, msg):
        """ Update the robot position from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.robot_yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

    def simulate_lidar_scan(self):
        """ Simulate a LiDAR scan using the static map """
        if self.robot_x is None or self.robot_y is None:
            rospy.logwarn("Odometry not yet received, skipping LiDAR scan...")
            return []

        scan_data = []
        angles = np.arange(self.angle_min, self.angle_max, self.angle_increment)

        for angle in angles:
            scan_range = self.lidar_range  # Start with max range
            ray_x, ray_y = self.robot_x, self.robot_y

            # Step along the ray to check for obstacles
            for i in range(int(self.lidar_range * 20)):  # 20 steps per meter
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
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = "base_link"
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.range_min = 0.1
        scan_msg.range_max = self.lidar_range
        scan_msg.ranges = self.simulate_lidar_scan()

        self.scan_pub.publish(scan_msg)

    def run(self):
        """ Main loop to publish LiDAR scans """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_scan()
            rate.sleep()

if __name__ == "__main__":
    try:
        FakeLidar()
    except rospy.ROSInterruptException:
        pass
