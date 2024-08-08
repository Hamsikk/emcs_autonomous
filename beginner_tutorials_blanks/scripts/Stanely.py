#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
from math import cos, sin, pi, sqrt, pow, atan2, atan, degrees, radians
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd
import numpy as np
from tf.transformations import euler_from_quaternion

class StanleyController:
    def __init__(self):
        rospy.init_node('stanley_controller', anonymous=True)
        rospy.Subscriber("local_path", Path, self.path_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        self.is_path = False
        self.is_odom = False
        self.vehicle_length = 4.635
        self.current_position = Point()
        self.vehicle_yaw = 0.0
        self.k = 0.5  # Stanley 조정 파라미터
        self.max_velocity = 12.0  # 최대 속도 (Stanley 사용 시)
        self.max_steering_angle = radians(40)  # 최대 조향각 (40도)
        self.min_steering_angle = radians(-40)  # 최소 조향각 (-40도)

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                self.stanley_control()
            else:
                os.system('clear')
                if not self.is_path:
                    print("[1] can't subscribe '/local_path' topic...")
                if not self.is_odom:
                    print("[2] can't subscribe '/odom' topic...")
            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y

    def stanley_control(self):
        # 경로 상의 가장 가까운 포인트 찾기
        min_dist = float('inf')
        nearest_point_index = -1
        for i, path_point in enumerate(self.path.poses):
            dx = path_point.pose.position.x - self.current_position.x
            dy = path_point.pose.position.y - self.current_position.y
            dist = sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                nearest_point_index = i

        if nearest_point_index == -1 or nearest_point_index >= len(self.path.poses) - 2:
            print("No valid forward point found")
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 0.0
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            return

        # Stanley 제어 계산
        point2 = self.path.poses[nearest_point_index + 1].pose.position
        dx = point2.x - self.current_position.x
        dy = point2.y - self.current_position.y

        theta_path = atan2(dy, dx)
        theta_e = theta_path - self.vehicle_yaw

        # 헤딩 오차 보정
        if theta_e > pi:
            theta_e -= 2 * pi
        elif theta_e < -pi:
            theta_e += 2 * pi

        e = sqrt(dx ** 2 + dy ** 2)

        v = self.max_velocity
        stanley_term = atan(self.k * e / (v + 0.1))

        steering_angle = theta_e + stanley_term

        # 스티어링 각도 제한
        steering_angle = max(min(steering_angle, self.max_steering_angle), self.min_steering_angle)

        self.ctrl_cmd_msg.steering = steering_angle
        self.ctrl_cmd_msg.velocity = v

        os.system('clear')
        print("Using Stanley")
        print("Steering (deg):", degrees(self.ctrl_cmd_msg.steering))
        print("Velocity (kph):", self.ctrl_cmd_msg.velocity)
        print("-------------------------------------")
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

if __name__ == '__main__':
    try:
        StanleyController()
    except rospy.ROSInterruptException:
        pass

