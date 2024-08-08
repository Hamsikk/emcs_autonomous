#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

class path_pub:

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.global_path2_pub = rospy.Publisher('/global_path2', Path, queue_size=1)
        self.local_path2_pub = rospy.Publisher('/local_path2', Path, queue_size=1)
        self.global_path2_msg = Path()
        self.global_path2_msg.header.frame_id = '/map'
        
        self.is_odom = False
        self.local_path2_size = 50

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('beginner_tutorials')
        full_path = pkg_path + '/path' + '/Seongnam_Cityhall_Global_Path2.txt'

        try:
            with open(full_path, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip()
                    if line:  # Skip empty lines
                        tmp = line.split()
                        if len(tmp) >= 2:  # Ensure there are at least two elements
                            read_pose = PoseStamped()
                            read_pose.pose.position.x = float(tmp[0])
                            read_pose.pose.position.y = float(tmp[1])
                            read_pose.pose.orientation.w = 1
                            self.global_path2_msg.poses.append(read_pose)
            rospy.loginfo("Successfully read global path from file")
        except Exception as e:
            rospy.logerr(f"Error reading path file: {e}")
            return

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            if self.is_odom:
                local_path2_msg = Path()
                local_path2_msg.header.frame_id = '/map'
                
                x = self.x
                y = self.y
                min_dis = float('inf')
                current_waypoint = -1
                for i, waypoint in enumerate(self.global_path2_msg.poses):
                    distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    end_index = min(current_waypoint + self.local_path2_size, len(self.global_path2_msg.poses))
                    for num in range(current_waypoint, end_index):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path2_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path2_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1
                        local_path2_msg.poses.append(tmp_pose)

                rospy.loginfo(f"Current position: x={x}, y={y}")
                self.global_path2_pub.publish(self.global_path2_msg)
                self.local_path2_pub.publish(local_path2_msg)

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
        test_track = path_pub()
    except rospy.ROSInterruptException:
        pass

