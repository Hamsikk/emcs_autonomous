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

class HybridController:
    def __init__(self):
        rospy.init_node('hybrid_controller', anonymous=True)
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
        self.max_velocity_pure_pursuit = 17.0  # Pure Pursuit 최대 속도
        self.max_velocity_stanley = 12.0  # Stanley 최대 속도
        self.max_steering_angle = radians(40)  # 최대 조향각 (40도)
        self.min_steering_angle = radians(-40)  # 최소 조향각 (-40도)
        self.lfd = 5  # Pure Pursuit look-ahead distance
        self.curvature_threshold = 0.1  # 곡률 임계값

        rate = rospy.Rate(15)  # 15hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom:
                self.hybrid_control()
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

    def calculate_curvature(self, point1, point2, point3):
        """
        세 점의 곡률을 계산합니다.
        """
        A = np.array([point1.x, point1.y])
        B = np.array([point2.x, point2.y])
        C = np.array([point3.x, point3.y])

        # 세 점이 직선상에 있는 경우 곡률은 0입니다.
        if np.linalg.norm(B - A) == 0 or np.linalg.norm(C - B) == 0:
            return 0

        # 원의 중심과 반지름을 계산합니다.
        a = np.linalg.norm(C - B)
        b = np.linalg.norm(A - C)
        c = np.linalg.norm(B - A)
        s = (a + b + c) / 2
        area = np.sqrt(s * (s - a) * (s - b) * (s - c))
        if area == 0:
            return 0
        radius = (a * b * c) / (4 * area)

        curvature = 1 / radius
        return curvature

    def hybrid_control(self):
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

        # 곡률 계산
        point1 = self.path.poses[nearest_point_index].pose.position
        point2 = self.path.poses[nearest_point_index + 1].pose.position
        point3 = self.path.poses[nearest_point_index + 2].pose.position
        curvature = self.calculate_curvature(point1, point2, point3)

        if curvature < self.curvature_threshold:
            # Pure Pursuit 사용
            self.pure_pursuit_control(point1, point2)
        else:
            # Stanley 사용
            self.stanley_control(point2)

    def pure_pursuit_control(self, point1, point2):
        translation = [self.current_position.x, self.current_position.y]
        t = np.array([
            [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
            [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
            [0, 0, 1]
        ])
        det_t = np.array([
            [t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])],
            [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],
            [0, 0, 1]
        ])

        self.is_look_forward_point = False
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

        if not self.is_look_forward_point:
            print("No forward point found")
            self.ctrl_cmd_msg.steering = 0.0
            self.ctrl_cmd_msg.velocity = 0.0
        else:
            dx = self.forward_point.x - self.current_position.x
            dy = self.forward_point.y - self.current_position.y

            theta = atan2(dy, dx) - self.vehicle_yaw
            if theta > pi:
                theta -= 2 * pi
            elif theta < -pi:
                theta += 2 * pi

            self.ctrl_cmd_msg.steering = atan2(2 * self.vehicle_length * sin(theta), self.lfd)
            self.ctrl_cmd_msg.steering = max(min(self.ctrl_cmd_msg.steering, self.max_steering_angle), self.min_steering_angle)
            self.ctrl_cmd_msg.velocity = self.max_velocity_pure_pursuit

        os.system('clear')
        print("Using Pure Pursuit")
        print("Steering (deg):", degrees(self.ctrl_cmd_msg.steering))
        print("Velocity (kph):", self.ctrl_cmd_msg.velocity)
        print("-------------------------------------")
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def stanley_control(self, point2):
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

        v = self.max_velocity_stanley
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
        HybridController()
    except rospy.ROSInterruptException:
        pass

