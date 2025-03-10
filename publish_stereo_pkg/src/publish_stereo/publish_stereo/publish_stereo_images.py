#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class KittiImagePublisher(Node):
    def __init__(self):
        super().__init__('kitti_image_publisher')
        self.get_logger().info("Image Publisher Node Started!")

        # Read parameters
        self.kitti_path = '/root/ACNMR/00'
        self.fps = 10

        # Directories for left and right images
        self.left_dir = os.path.join(self.kitti_path, 'image_2')
        self.right_dir = os.path.join(self.kitti_path, 'image_3')

        # Verify directories
        if not os.path.exists(self.left_dir) or not os.path.exists(self.right_dir):
            self.get_logger().error(f"Invalid KITTI dataset path: {self.kitti_path}")
            raise FileNotFoundError("KITTI dataset directories not found!")

        # Get sorted image filenames
        self.left_images = sorted(os.listdir(self.left_dir))
        self.right_images = sorted(os.listdir(self.right_dir))

        # Ensure same number of images in both directories
        if len(self.left_images) != len(self.right_images):
            self.get_logger().error("Mismatch in number of images between left and right cameras.")
            raise ValueError("Left and right image counts do not match!")

        # ROS 2 publishers
        self.left_pub = self.create_publisher(Image, '/left/image_rect', 10)
        self.right_pub = self.create_publisher(Image, '/right/image_rect', 10)

        # CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Timer to publish images at the specified FPS
        self.index = 0
        self.timer = self.create_timer(1.0 / self.fps, self.publish_images)

        self.get_logger().info(f"KITTI Image Publisher started with {len(self.left_images)} images at {self.fps} FPS.")

    def publish_images(self):
        if self.index >= len(self.left_images):
            self.get_logger().info("All images published. Restarting sequence...")
            self.index = 0  # Loop back to start

        # Load left and right images
        left_img_path = os.path.join(self.left_dir, self.left_images[self.index])
        right_img_path = os.path.join(self.right_dir, self.right_images[self.index])

        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)

        if left_img is None or right_img is None:
            self.get_logger().error(f"Failed to load images: {left_img_path}, {right_img_path}")
            return

        # Convert to ROS Image messages
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')

        # Publish images
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.get_logger().info(f"Published frame {self.index}: {self.left_images[self.index]}, {self.right_images[self.index]}")

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = KittiImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
