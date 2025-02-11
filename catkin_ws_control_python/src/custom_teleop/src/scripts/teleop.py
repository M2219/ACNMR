#!/usr/bin/env python3
from __future__ import print_function

import threading
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import roslib
import rospy

import termios
import tty

import numpy as np
import math

import cubic_spline_planner as cubic_spline_planner

from select import select

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from std_msgs.msg import Float64

from pure_pursuit import TargetCourse, pure_pursuit_steer_control
from stanley import stanley_control, calc_target_index

from scipy.spatial.transform import Rotation as Rot
from mpc_utils import *
from model_predictive_speed_and_steer_control import MPC

import tf.transformations

TwistMsg = Twist

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, WB=0.650):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.WB = WB
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta, dt):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)

        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.odom_lock = threading.Lock()
        self.done = False

        self.x_odo = 0.0
        self.y_odo = 0.0
        self.v_odo = 0.0
        self.yaw_odo = 0.0

        self.max_steer_angle = 0.58
        self.max_steer_angle_central = 0.461
        self.max_linear_speed = 1.5
        self.L = 0.650

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.target_x_publisher = rospy.Publisher('/target_x', Float64, queue_size=1)
        self.target_y_publisher = rospy.Publisher('/target_y', Float64, queue_size=1)
        self.target_yaw_publisher = rospy.Publisher('/target_yaw', Float64, queue_size=1)
        self.target_ttime_publisher = rospy.Publisher('/target_time', Float64, queue_size=1)

        self.target_x_test = rospy.Publisher('/test_x', Float64, queue_size=1)
        self.target_y_test = rospy.Publisher('/test_y', Float64, queue_size=1)

        self.dt = 1.0 / 25

        # pure pursuit initialization
        self.pure_pursuit = False
        if self.pure_pursuit:
            self.target_speed = 0.2
            self.delta_filtered = 0
            self.cx = np.arange(0, 5000, 0.02)
            #self.cy = np.arange(0, 50, 0.2)
            self.cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in self.cx]

            self.lastIndex = len(self.cx) - 1
            self.target_course = TargetCourse(self.cx, self.cy, k=1, Lfc=self.L / 2) # common guidline self.L / 2
            self.rear_x = self.x_odo - ((self.L / 2) * math.cos(self.yaw_odo))
            self.rear_y = self.y_odo - ((self.L / 2) * math.sin(self.yaw_odo))

            self.state = State(x=0.0, y=0.0, yaw=0.0, v=0.0, WB=self.L)
            self.target_ind, _ = self.target_course.search_target_index(self.rear_x, self.rear_y, self.v_odo)

        self.stanley_control = False
        if self.stanley_control:
            self.target_speed = 1
            self.k_stanley = 0.05
            ax = [0.0, 100.0, 100.0, 50.0, 60.0]
            ay = [0.0, 0.0, -30.0, -20.0, 0.0]
            #ax = [0.0, 10.0, 20.0, 30.0, 60.0]
            #ay = [0.0, 10.0, 20.0, 30.0, 60.0]
            #ax = [0.0, 10.0, 20.0, 30.0, 40.0]
            #ay = [0.0, 5.0, 15.0, 30.0, 50.0]  # More curvature

            self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.02)
            self.target_ind, _ = calc_target_index(self.cx, self.cy, self.x_odo, self.y_odo, self.yaw_odo, self.L)

        self.mpc_control = True
        if self.mpc_control:
            self.target_speed = 1
            self.dl = 0.1 #1.0 / 25
            #self.cx, self.cy, self.cyaw, self.ck = get_straight_course3(self.dl)
            #self.cx, self.cy, self.cyaw, self.ck = get_straight_course2(self.dl)
            #self.cx, self.cy, self.cyaw, self.ck = get_forward_course(self.dl)
            self.cx, self.cy, self.cyaw, self.ck = get_switch_back_course(self.dl)
            self.sp = calc_speed_profile(self.cx, self.cy, self.cyaw, self.target_speed)
            self.state = State(x=0, y=0, yaw=0, v=0, WB=self.L)

            self.N_IND_SEARCH = 10
            self.target_ind, _ = calc_nearest_index(self.state, self.cx, self.cy, self.cyaw, 0, self.N_IND_SEARCH)

            # initial yaw compensation - also consider it in odometry
            if self.state.yaw - self.cyaw[0] >= math.pi:
                self.state.yaw -= math.pi * 2.0
                self.yaw_comp = -math.pi * 2.0
            elif self.state.yaw - self.cyaw[0] <= -math.pi:
                self.state.yaw += math.pi * 2.0
                self.yaw_comp = math.pi * 2.0
            else:
                self.yaw_comp = 0

            self.goal = [self.cx[-1], self.cy[-1]]

            self.T = 5
            self.oa = [0.0] * self.T
            self.odelta = [0.0] * self.T
            self.ai = 0
            self.di = 0
            self.cyaw = smooth_yaw(self.cyaw)

            MAX_STEER = self.max_steer_angle # maximum steering angle [rad]
            MAX_DSTEER = np.deg2rad(40.0)  # maximum steering speed [rad/s]
            MAX_SPEED = self.max_linear_speed # maximum speed [m/s]
            MIN_SPEED = -self.max_linear_speed # minimum speed [m/s]
            MAX_ACCEL = 1.0  # maximum accel [m/ss]

            self.NX = 4 # number of states
            self.mpc_controller = MPC(self.dt, self.N_IND_SEARCH, self.T, self.NX, self.L, MAX_STEER, MAX_DSTEER, MAX_SPEED, MIN_SPEED, MAX_ACCEL)
            self.GOAL_DIS = 1.5  # goal distance
            self.STOP_SPEED = 0.125 # stop speed
            self.odom_updated = False

        self.propotional = False
        self.p_gain = 2.5
        self.PID = False
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.02
        self.integral = 0.0
        self.prev_error = 0.0

        self.start()

    def publish_target_test(self, x, y):
        """ Publishes x and y as individual topics """

        x_msg = Float64()
        x_msg.data = x
        self.target_x_test.publish(x_msg)

        y_msg = Float64()
        y_msg.data = y
        self.target_y_test.publish(y_msg)

        #z_msg = Float64()
        #z_msg.data = yaw
        #self.target_yaw_publisher.publish(z_msg)

        #t_msg = Float64()
        #t_msg.data = ttime
        #self.target_ttime_publisher.publish(t_msg)

    def publish_target_trajectory(self, x, y, yaw, ttime):
        """ Publishes x and y as individual topics """

        x_msg = Float64()
        x_msg.data = x
        self.target_x_publisher.publish(x_msg)

        y_msg = Float64()
        y_msg.data = y
        self.target_y_publisher.publish(y_msg)

        z_msg = Float64()
        z_msg.data = yaw
        self.target_yaw_publisher.publish(z_msg)

        t_msg = Float64()
        t_msg.data = ttime
        self.target_ttime_publisher.publish(t_msg)


    def odom_callback(self, msg):
        """Extracts odometry information from /odom topic"""
        with self.odom_lock:
            self.x_odo = msg.pose.pose.position.x
            self.y_odo = msg.pose.pose.position.y
            self.v_odo = msg.twist.twist.linear.x

            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )

            euler = tf.transformations.euler_from_quaternion(quaternion)

            if self.mpc_control:
                self.yaw_odo = euler[2] + self.yaw_comp
            else:
                self.yaw_odo = euler[2]


            #self.yaw_odo = msg.pose.pose.orientation.z
            self.odom_updated = True  # Flag for main loop

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self):
        self.condition.acquire()
        self.speed = self.speed
        self.turn = self.turn

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.condition.acquire()
        self.condition.notify()
        self.condition.release()
        self.join()

    def proportional_controller(self, speed_target):
        with self.odom_lock:
            error = speed_target - self.v_odo
            correction = self.p_gain * error
            speed_command = self.v_odo + self.dt * correction
            return speed_command

    def pid_controller(self, speed_target):
        with self.odom_lock:
            error = speed_target - self.v_odo
            self.integral += error * self.dt
            derivative = (error - self.prev_error) / self.dt

            correction = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error

            speed_command = self.v_odo + correction
            return speed_command

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
            twist = twist_msg.twist
        else:
            twist = twist_msg

        ttime = 0
        try:
           while not rospy.is_shutdown() and not self.done:

                if stamped:
                    twist_msg.header.stamp = rospy.Time.now()

                self.condition.acquire()
                self.condition.wait(self.timeout)

                if self.propotional:
                    self.speed = self.proportional_controller(self.target_speed)

                if self.PID:
                    self.speed = self.pid_controller(self.target_speed)


                if self.pure_pursuit:
                    self.turn, self.target_ind = pure_pursuit_steer_control(self.L, self.target_course, self.target_ind, self.rear_x, self.rear_y,
                                                                      self.x_odo, self.y_odo, self.v_odo, self.yaw_odo, self.delta_filtered)

                    #self.turn, self.target_ind = pure_pursuit_steer_control(self.L, self.target_course, self.target_ind, self.state.rear_x, self.state.rear_y,
                    #                                                  self.state.x, self.state.y, self.state.v, self.state.yaw, self.delta_filtered)

                    #self.state.update(1*(self.target_speed - self.state.v), self.turn, self.dt)  # Control vehicle

                    self.rear_x = self.x_odo - ((self.L / 2) * math.cos(self.yaw_odo))
                    self.rear_y = self.y_odo - ((self.L / 2) * math.sin(self.yaw_odo))

                    ttime = ttime + self.dt
                    #self.publish_target_test(self.state.x, self.state.y, self.turn, ttime)


                    if self.target_ind < len(self.cx):
                        self.publish_target_trajectory(self.cx[self.target_ind], self.cy[self.target_ind], self.turn, ttime)

                if self.stanley_control:
                    self.turn, self.target_ind = stanley_control(self.L, self.cx, self.cy, self.cyaw, self.target_ind, self.k_stanley,
                                                           self.yaw_odo, self.v_odo, self.x_odo, self.y_odo)

                    ttime = ttime + self.dt
                    print(self.turn)

                    if self.target_ind < len(self.cx):
                        self.publish_target_trajectory(self.cx[self.target_ind], self.cy[self.target_ind], self.turn, ttime)




                if self.mpc_control:
                    with self.odom_lock:
                        if not self.odom_updated:
                            rospy.logwarn("Waiting for new odometry before updating teleop commands...")
                            rospy.sleep(0.01)  # Small wait to allow /odom to update
                            continue  # Skip this iteration

                    xref, self.target_ind, dref = calc_ref_trajectory(self.state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl,
                                                                     self.target_ind, self.T, self.NX, self.N_IND_SEARCH, self.dt)

                    x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state

                    self.oa, self.odelta = self.mpc_controller.control_signals(xref, dref, self.oa, self.odelta, x0)

                    self.ai = self.oa[0]
                    self.di = self.odelta[0]
                    self.turn = self.di

                    print("v ", self.state.v, self.v_odo, self.target_ind, self.odom_updated)
                    print("x ", self.state.x, self.x_odo, self.odom_updated)
                    print("y ", self.state.y, self.y_odo, self.odom_updated)
                    print("yaw ", self.state.yaw, self.yaw_odo, self.odom_updated)

                    #self.speed = self.state.v  + self.dt * self.ai
                    self.speed = self.v_odo + self.dt * self.ai

                    #self.odom_updated = False  # Mark as processed
                    #self.state = self.mpc_controller.update_state(self.state, self.ai, self.di) #  this is actual model

                    self.state.x = self.x_odo
                    self.state.y = self.y_odo
                    self.state.v = self.v_odo
                    self.state.yaw = self.yaw_odo

                    #if ttime > 0.5:
                    #    self.state.yaw = self.yaw_odo

                    if not rospy.is_shutdown():
                        self.publish_target_test(self.state.x, self.state.y)

                    ttime = ttime + self.dt
                    if self.target_ind < len(self.cx):
                        if not rospy.is_shutdown():
                            self.publish_target_trajectory(self.cx[self.target_ind], self.cy[self.target_ind], self.di, ttime)

                # break command
                #if check_goal(state, goal, target_ind, len(cx), GOAL_DIS, STOP_SPEED):
                #     print("Goal")
                #     break

                # Stop if the last waypoint is reached
                # if np.linalg.norm(np.array([x, y]) - waypoints[-1]) < 0.5:
                #    break
                """
                if self.turn > self.max_steer_angle_central:
                    self.turn = self.max_steer_angle_central
                if self.turn < - self.max_steer_angle_central:
                    self.turn = - self.max_steer_angle_central

                if self.speed > self.max_linear_speed:
                    self.speed = self.max_linear_speed
                if self.speed < - self.max_linear_speed:
                    self.speed = - self.max_linear_speed

                if self.speed == self.max_linear_speed:
                    print("Linear speed limit reached!")
                if self.turn == self.max_steer_angle_central:
                    print("Angular speed limit reached!")
                """
                """
                twist.linear.x = self.speed
                twist.linear.y = 0
                twist.linear.z = 0

                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.turn

                self.condition.release()
                self.publisher.publish(twist_msg)
                """
                twist.linear.x = self.speed
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = self.turn

                self.publisher.publish(twist_msg)
                self.condition.release()

        except KeyboardInterrupt:
           rospy.loginfo("Shutting down due to Ctrl+C...")

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__=="__main__":

    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist')

    repeat = rospy.get_param("~repeat_rate", 50.0)
    repeat = 25
    rate = rospy.Rate(repeat)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')

    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)
    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update()
        while not rospy.is_shutdown():
            pub_thread.update()
            rate.sleep()

    except Exception as e:
        print(e)
        pub_thread.stop()
        restoreTerminalSettings(settings)
        rospy.signal_shutdown("Teleop node stopped")
        os._exit(0)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
        rospy.signal_shutdown("Teleop node stopped")
        os._exit(0)

