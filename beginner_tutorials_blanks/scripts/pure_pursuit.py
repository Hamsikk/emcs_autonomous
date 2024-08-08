#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from math import cos, sin, sqrt, pow, atan2
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
import numpy as np
from tf.transformations import euler_from_quaternion

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)
        rospy.Subscriber("/lattice_path", Path, self.lattice_path_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_lattice_path = False
        self.is_local_path = False
        self.is_odom = False

        self.last_lattice_path_time = rospy.Time.now()
        self.lattice_path_timeout = 2.0  # seconds

        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 4.635
        self.lfd = 4

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if (self.is_lattice_path or self.is_local_path) and self.is_odom:
                current_time = rospy.Time.now()
                if self.is_lattice_path and (current_time - self.last_lattice_path_time).to_sec() > self.lattice_path_timeout:
                    self.is_lattice_path = False  # lattice_path가 타임아웃되었으면 false로 설정

                if self.is_lattice_path and self.reached_goal(self.lattice_path):
                    self.is_lattice_path = False  # lattice_path를 다 따라갔으면 local_path로 전환

                vehicle_position = self.current_position
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

                path_to_follow = self.lattice_path if self.is_lattice_path else self.local_path
                path_type = "lattice_path" if self.is_lattice_path else "local_path"

                for num, i in enumerate(path_to_follow.poses):
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
                    if self.ctrl_cmd_msg.steering is None:
                        print("you need to change the value at line 70")
                        exit()
                    self.ctrl_cmd_msg.velocity = 5.0

                    os.system('clear')
                    print("-------------------------------------")
                    print(f"Following {path_type}")
                    print(" steering (deg) = ", self.ctrl_cmd_msg.steering * 180 / 3.14)
                    print(" velocity (kph) = ", self.ctrl_cmd_msg.velocity)
                    print("-------------------------------------")
                else:
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_msg.velocity = 0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            else:
                os.system('clear')
                if not self.is_lattice_path and not self.is_local_path:
                    print("[1] can't subscribe '/lattice_path' and '/local_path' topics...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")

            self.is_odom = False
            rate.sleep()

    def reached_goal(self, path):
        if not path.poses:
            return False

        goal_position = path.poses[-1].pose.position

        distance_to_goal = sqrt(pow(self.current_position.x - goal_position.x, 2) +
                                pow(self.current_position.y - goal_position.y, 2))

        return distance_to_goal < 0.05  # 차량이 목표 지점에 m 이내로 접근하면 경로 변경

    def lattice_path_callback(self, msg):
        self.is_lattice_path = True
        self.lattice_path = msg
        self.last_lattice_path_time = rospy.Time.now()  # lattice_path가 업데이트된 시간을 기록

    def local_path_callback(self, msg):
        self.is_local_path = True
        self.local_path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y


if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass

