#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import numpy as np
from math import cos, sin, tan, atan, pi, sqrt, radians, degrees
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage, CtrlCmd
from nav_msgs.msg import Odometry
from pyproj import Proj
from tf.transformations import euler_from_quaternion

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.ctrl_cmd_sub = rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_cmd_callback)  # 조향각과 속도 데이터 구독
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        self.x, self.y = None, None
        self.last_valid_x, self.last_valid_y = None, None
        self.is_imu = False
        self.is_gps = False
        self.last_time = rospy.Time.now()

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'
        self.odom_msg.child_frame_id = '/base_link'

        self.velocity = 0.0  # m/s 단위의 속도
        self.yaw = 0.0
        self.previous_yaw = 0.0  # 이전 yaw 값을 저장할 변수 추가
        self.steering_angle = 0.0  # 조향각 (라디안 단위)

        self.wheelbase = 3 # 차량의 휠베이스 (미터 단위)
        self.lr = self.wheelbase / 1.5  # 후륜 축까지의 거리

        rate = rospy.Rate(10)  # 주기를 10Hz로 설정하여 더 자주 위치를 업데이트
        while not rospy.is_shutdown():
            os.system('clear')
            if self.is_imu and (self.is_gps or self.last_valid_x is not None and self.last_valid_y is not None):
                self.update_position()
                self.odom_pub.publish(self.odom_msg)
                self.print_status()

            if not self.is_imu:
                print("[1] can't subscribe '/imu' topic... \n    please check your IMU sensor connection")
            if not self.is_gps and (self.x is None and self.y is None):
                print("[2] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")

            rate.sleep()

    def print_status(self):
        print(f"odom_msg is now being published at '/odom' topic!\n")
        print('-----------------[ odom_msg ]---------------------')
        print(self.odom_msg.pose)
        print(f"Current GPS Position: ({self.x}, {self.y})")
        print(f"Current Velocity: {self.velocity/ 0.27778} km/s")
        print(f"Current Yaw: {degrees(self.yaw)} degrees")
        print(f"Previous Yaw: {degrees(self.previous_yaw)} degrees")
        print(f"Steering Angle: {degrees(self.steering_angle)} degrees")

    def navsat_callback(self, gps_msg):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        if self.lon != 0 and self.lat != 0:
            self.is_gps = True
            xy_zone = self.proj_UTM(self.lon, self.lat)
            self.last_valid_x = xy_zone[0] - self.e_o
            self.last_valid_y = xy_zone[1] - self.n_o
            self.x = self.last_valid_x
            self.y = self.last_valid_y
        else:
            self.is_gps = False

    def update_position(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        delta_x, delta_y = 0, 0

        if self.is_gps:
            xy_zone = self.proj_UTM(self.lon, self.lat)
            self.x = xy_zone[0] - self.e_o
            self.y = xy_zone[1] - self.n_o
            # 마지막 유효 GPS 위치 업데이트
            self.last_valid_x = self.x
            self.last_valid_y = self.y
            self.previous_yaw = self.yaw  # GPS 신호가 유효할 때 이전 yaw 값 업데이트
        elif self.last_valid_x is not None and self.last_valid_y is not None:
            # GPS 데이터가 0인 경우 마지막 유효한 GPS 위치를 사용하고 IMU와 속도로 위치 추정
            delta_yaw = self.yaw - self.previous_yaw  # yaw 변화량 계산

            if abs(delta_yaw) > pi:
                if delta_yaw > 0:
                    delta_yaw -= 2 * pi
                else:
                    delta_yaw += 2 * pi

            corrected_yaw = self.previous_yaw + delta_yaw  # yaw 보정

            # Body side slip angle 계산
            body_side_slip = atan(self.lr * tan(self.steering_angle) / self.wheelbase)

            delta_x = self.velocity * dt * cos(corrected_yaw + body_side_slip)
            delta_y = self.velocity * dt * sin(corrected_yaw + body_side_slip)

            self.x = self.last_valid_x + delta_x
            self.y = self.last_valid_y + delta_y

            # 추정된 위치를 업데이트
            self.last_valid_x = self.x
            self.last_valid_y = self.y
            self.previous_yaw = corrected_yaw  # 보정된 yaw 값 저장

        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

        # 위치 변경 로그 출력
        print(f"Updated Position -> X: {self.x}, Y: {self.y}, Delta_X: {delta_x}, Delta_Y: {delta_y}, DT: {dt}")

    def imu_callback(self, data):
        if data.orientation.w != 0:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w

            orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
            _, _, self.yaw = euler_from_quaternion(orientation_list)

        self.is_imu = True

    def ctrl_cmd_callback(self, ctrl_cmd_msg):
        self.velocity = ctrl_cmd_msg.velocity * 0.27778  # km/h 단위의 속도를 m/s 단위로 변환
        self.steering_angle = ctrl_cmd_msg.steering # 라디안 단위로 변환

if __name__ == '__main__':
    try:
        GPSIMUParser()
    except rospy.ROSInterruptException:
        pass

