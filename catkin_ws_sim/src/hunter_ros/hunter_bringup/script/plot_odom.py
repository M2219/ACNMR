#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry

# Initialize lists for x and y positions
x_data, y_data = [], []

def odom_callback(msg):
    """Callback function to receive odometry data and update position lists."""
    global x_data, y_data

    # Extract position
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Append data for real-time plotting
    x_data.append(x)
    y_data.append(y)

    # Keep only the last 100 points to prevent lag
    if len(x_data) > 100:
        x_data.pop(0)
        y_data.pop(0)

def update_plot(event):
    """Timer function to update the plot periodically."""
    plt.cla()  # Clear previous plot
    plt.plot(x_data, y_data, 'b-', label="Robot Path")
    if x_data and y_data:
        plt.scatter(x_data[-1], y_data[-1], color='r', label="Current Position")  # Mark last position

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Live Odometry Plot")
    plt.legend()
    plt.grid(True)
    plt.draw()  # Update the plot

def main():
    rospy.init_node('odom_plotter', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Setup Matplotlib figure
    plt.ion()  # Enable interactive mode
    fig = plt.figure()

    # Use a ROS Timer to update the plot instead of plt.pause()
    timer = fig.canvas.new_timer(interval=100)  # 100ms interval
    timer.add_callback(update_plot, None)
    timer.start()

    rospy.spin()  # Keep the node running
    plt.ioff()  # Disable interactive mode when node shuts down
    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
