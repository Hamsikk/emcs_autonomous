#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os, time
from pyproj import Proj
from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage

class GPS_to_UTM:
    def __init__(self):
        rospy.init_node('GPS_to_UTM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        self.map_pub = rospy.Publisher("/map_coordinates", Float32MultiArray, queue_size=10)
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False
        self.map_x = None
        self.map_y = None

        # Open the file in append mode
        self.file = open("gps_data.txt", "a")

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_gps_data:
                print("[1] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")

            self.is_gps_data = False
            rate.sleep()

    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        utm_xy = self.proj_UTM(longitude, latitude)
        utm_x = utm_xy[0]
        utm_y = utm_xy[1]
        self.map_x = utm_x - gps_msg.eastOffset
        self.map_y = utm_y - gps_msg.northOffset

        os.system('clear')
        print(f'''
        ----------------[ GPS data ]----------------
            latitude    : {latitude}
            longitude   : {longitude}
            altitude    : {altitude}

                             |
                             | apply Projection (utm 52 zone)
                             V

        ------------------[ utm ]-------------------
              utm_x     : {utm_x}
              utm_y     : {utm_y}

                             |
                             | apply offset (east and north)
                             V
              
        ------------------[ map ]-------------------
        simulator map_x : {self.map_x}
        simulator map_y : {self.map_y}
        ''')

        # 퍼블리시할 메시지 설정
        self.utm_msg.data = [self.map_x, self.map_y]
        self.map_pub.publish(self.utm_msg)

        # Save the data to the file every second
        current_time = time.time()
        if hasattr(self, 'last_save_time'):
            if current_time - self.last_save_time >= 1.0:
                self.save_to_file()
                self.last_save_time = current_time
        else:
            self.last_save_time = current_time

    def save_to_file(self):
        if self.map_x is not None and self.map_y is not None:
            self.file.write(f"{self.map_x}, {self.map_y}\n")
            self.file.flush()

    def __del__(self):
        if self.file:
            self.file.close()

if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_UTM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

