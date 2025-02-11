import rospy
import message_filters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Callback function for synchronized messages
def callback(cmd_vel_msg, odom_msg):
    rospy.loginfo(f"Synchronized cmd_vel: {cmd_vel_msg.linear.x}, odom: {odom_msg.twist.twist.linear.x}")

def main():
    rospy.init_node('sync_example')

    # Create subscribers with the correct message types
    cmd_vel_sub = message_filters.Subscriber('/cmd_vel', Twist)
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    # Synchronize using ApproximateTimeSynchronizer
    ts = message_filters.ApproximateTimeSynchronizer([cmd_vel_sub, odom_sub], queue_size=2, slop=0.001, allow_headerless=True) # remove allow_headerless=True and add header.stamp to cmd_vel
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    main()
