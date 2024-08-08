#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs.point_cloud2 as pc2

class ClusterToGlobal:
    def __init__(self):
        rospy.init_node('cluster_to_global', anonymous=True)
        
        self.cluster_sub = rospy.Subscriber("/cluster_points", PointCloud2, self.cluster_callback)
        self.map_sub = rospy.Subscriber("/map_coordinates", Float32MultiArray, self.map_callback)
        self.yaw_sub = rospy.Subscriber("/yaw_deg", Float32, self.yaw_callback)
        
        self.cluster_global_pub = rospy.Publisher("/cluster_global", PoseArray, queue_size=10)
        
        self.current_map = None
        self.current_yaw = None

    def map_callback(self, map_msg):
        self.current_map = map_msg.data
        rospy.loginfo(f"Received map coordinates: x={self.current_map[0]}, y={self.current_map[1]}")

    def yaw_callback(self, yaw_msg):
        self.current_yaw = np.radians(yaw_msg.data)
        rospy.loginfo(f"Received Yaw data: yaw={yaw_msg.data} degrees")

    def cluster_callback(self, cloud_msg):
        if self.current_map is None or self.current_yaw is None:
            rospy.logwarn("Waiting for map coordinates and Yaw data...")
            return

        points = self.pointcloud2_to_xyz(cloud_msg)
        if len(points) == 0:
            return

        cluster_global = self.convert_to_global(points)
        self.publish_global_clusters(cluster_global)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2]))

        return np.array(point_list, np.float32)

    def convert_to_global(self, cluster_points):
        cluster_global = PoseArray()
        cluster_global.header.frame_id = "global"
        cluster_global.header.stamp = rospy.Time.now()

        delta_x, delta_y = self.current_map

        for c in cluster_points:
            local_x, local_y = c[:2]
            theta = self.current_yaw

            global_x = (local_x * np.cos(theta) - local_y * np.sin(theta)) + delta_x
            global_y = (local_x * np.sin(theta) + local_y * np.cos(theta)) + delta_y

            pose = Pose()
            pose.position.x = global_x
            pose.position.y = global_y
            pose.position.z = c[2] 

            cluster_global.poses.append(pose)

            rospy.loginfo(f"Cluster local coordinates: x={local_x}, y={local_y}, Global coordinates: x={global_x}, y={global_y}")

        return cluster_global

    def publish_global_clusters(self, cluster_global):
        self.cluster_global_pub.publish(cluster_global)
        rospy.loginfo("Published global cluster positions")

if __name__ == '__main__':
    try:
        ClusterToGlobal = ClusterToGlobal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

