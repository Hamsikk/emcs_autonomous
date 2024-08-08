#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import String
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from threading import Timer

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("local_path", Path, self.local_path_callback)
        rospy.Subscriber("local_path2", Path, self.local_path2_callback)
        rospy.Subscriber("/obstacle_path", String, self.obstacle_path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_local_path = False
        self.is_local_path2 = False
        self.is_odom = False
        self.is_path = False

        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 4.635
        self.lfd = 6

        self.target_velocity = 13.0
        self.current_velocity = 0.0

        self.kp = 2.0
        self.ki = 0.1
        self.kd = 2.0
        self.prev_error = 0
        self.integral = 0

        self.current_path = "local_path"
        self.can_change_path = True
        self.change_path_timer = None
        self.reset_to_local_path_timer = None

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                self.pure_pursuit_control()
            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe path topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")

            self.is_path = False
            self.is_odom = False
            rate.sleep()

    def pure_pursuit_control(self):
        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        translation = [vehicle_position.x, vehicle_position.y]

        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]])

        det_t = np.array([
            [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
            [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
            [0, 0, 1]])

        for num, i in enumerate(self.path.poses):
            path_point = i.pose.position

            global_path_point = [path_point.x, path_point.y, 1]
            local_path_point = det_t.dot(global_path_point)
            if local_path_point[0] > 0:
                dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                if dis >= self.lfd:
                    self.forward_point = path_point
                    self.is_look_forward_point = True
                    break

        theta = atan2(local_path_point[1], local_path_point[0])

        if self.is_look_forward_point:
            self.ctrl_cmd_msg.steering = atan2((2 * self.vehicle_length * sin(theta)), self.lfd)
            self.apply_pid_control()
            os.system('clear')
            print("-------------------------------------")
            print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180 / 3.14)
            print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
            print("-------------------------------------")
        else:
            print("no found forward point")
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 0.0

        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def local_path_callback(self, msg):
        self.is_local_path = True
        self.local_path = msg
        if self.current_path == "local_path":
            self.is_path = True
            self.path = msg

    def local_path2_callback(self, msg):
        self.is_local_path2 = True
        self.local_path2 = msg
        if self.current_path == "local_path2":
            self.is_path = True
            self.path = msg

    def obstacle_path_callback(self, msg):
        if not self.can_change_path:
            return

        if self.current_path == "local_path" and msg.data == "local_path":
            self.current_path = "local_path2"
            if self.is_local_path2:
                self.is_path = True
                self.path = self.local_path2
            if self.reset_to_local_path_timer:
                self.reset_to_local_path_timer.cancel()
            self.reset_to_local_path_timer = Timer(5, self.reset_to_local_path)
            self.reset_to_local_path_timer.start()

        self.can_change_path = False
        if self.change_path_timer:
            self.change_path_timer.cancel()
        self.change_path_timer = Timer(1, self.reset_path_change_flag)
        self.change_path_timer.start()

    def reset_to_local_path(self):
        self.current_path = "local_path"
        if self.is_local_path:
            self.is_path = True
            self.path = self.local_path

    def reset_path_change_flag(self):
        self.can_change_path = True

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                           msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
        self.current_velocity = msg.twist.twist.linear.x

    def apply_pid_control(self):
        error = self.target_velocity - self.current_velocity
        self.integral += error
        derivative = error - self.prev_error

        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.ctrl_cmd_msg.velocity = max(0.0, min(self.target_velocity, self.current_velocity + control_output))
        self.prev_error = error


if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass

