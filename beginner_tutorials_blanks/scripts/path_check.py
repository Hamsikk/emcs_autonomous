#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

class path_pub:

    def __init__(self):
        rospy.init_node('path_pub', anonymous=True)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cluster_global", PoseArray, self.cluster_callback)
        rospy.Subscriber("local_path", Path, self.local_path_callback)
        rospy.Subscriber("local_path2", Path, self.local_path2_callback)  
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.obstacle_path_pub = rospy.Publisher('/obstacle_path', String, queue_size=1)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        
        self.is_odom = False
        self.is_cluster = False
        self.is_local_path = False
        self.is_local_path2 = False  
        self.local_path_size = 50
        self.obstacle_threshold = 3.0

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('beginner_tutorials')
        full_path = pkg_path + '/path' + '/Seongnam_Cityhall_Global_Path.txt'

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
                            self.global_path_msg.poses.append(read_pose)
            rospy.loginfo("Successfully read global path from file")
        except Exception as e:
            rospy.logerr(f"Error reading path file: {e}")
            return

        rate = rospy.Rate(20)  # 20hz
        while not rospy.is_shutdown():
            if self.is_odom:
                local_path_msg = Path()
                local_path_msg.header.frame_id = '/map'
                
                x = self.x
                y = self.y
                min_dis = float('inf')
                current_waypoint = -1
                for i, waypoint in enumerate(self.global_path_msg.poses):
                    distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

                if current_waypoint != -1:
                    end_index = min(current_waypoint + self.local_path_size, len(self.global_path_msg.poses))
                    for num in range(current_waypoint, end_index):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1
                        local_path_msg.poses.append(tmp_pose)

                rospy.loginfo(f"Current position: x={x}, y={y}")
                self.global_path_pub.publish(self.global_path_msg)

            if self.is_cluster:
                self.check_obstacles_on_paths()

            rate.sleep()

    def odom_callback(self, msg):
        self.is_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def cluster_callback(self, msg):
        self.is_cluster = True
        self.cluster_points = msg.poses

    def local_path_callback(self, msg):
        self.is_local_path = True
        self.local_path_msg = msg

    def local_path2_callback(self, msg):
        self.is_local_path2 = True
        self.local_path2_msg = msg

    def check_obstacles_on_paths(self):
        local_path_obstacle_detected = False
        local_path2_obstacle_detected = False

        if self.is_local_path:
            local_path_obstacle_detected = self.check_obstacles_on_path(self.local_path_msg, "local_path")

        if self.is_local_path2:  # 수정된 부분
            local_path2_obstacle_detected = self.check_obstacles_on_path(self.local_path2_msg, "local_path2")

        if local_path_obstacle_detected:
            self.obstacle_path_pub.publish("local_path")
        elif local_path2_obstacle_detected:
            self.obstacle_path_pub.publish("local_path2")
        else:
            self.obstacle_path_pub.publish("no_obstacle")

        if not local_path_obstacle_detected and not local_path2_obstacle_detected:
            rospy.loginfo("No obstacles detected on either path")

    def check_obstacles_on_path(self, path_msg, path_name):
        obstacle_detected = False
        for cluster in self.cluster_points:
            cluster_x = cluster.position.x
            cluster_y = cluster.position.y
            for waypoint in path_msg.poses:
                path_x = waypoint.pose.position.x
                path_y = waypoint.pose.position.y
                distance = sqrt(pow(cluster_x - path_x, 2) + pow(cluster_y - path_y, 2))
                if distance < self.obstacle_threshold:
                    rospy.logwarn(f"Obstacle detected near {path_name}: cluster_x={cluster_x}, cluster_y={cluster_y}, distance={distance}")
                    obstacle_detected = True
                    break
            if obstacle_detected:
                break
        return obstacle_detected

if __name__ == '__main__':
    try:
        test_track = path_pub()
    except rospy.ROSInterruptException:
        pass

