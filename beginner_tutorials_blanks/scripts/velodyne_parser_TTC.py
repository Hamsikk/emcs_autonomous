#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int32
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Twist

class LidarCollisionDetector:
    def __init__(self):
        rospy.init_node('velodyne_clustering', anonymous=True)
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.ctrl_cmd_sub = rospy.Subscriber('/ctrl_cmd', Twist, self.ctrl_cmd_callback)
        self.clusterpoints_pub = rospy.Publisher("/cluster_points", PointCloud2, queue_size=10)
        self.collision_pub = rospy.Publisher('/collision_warning', Int32, queue_size=10)

        self.pc_np = None
        self.dbscan = DBSCAN(eps=0.5, min_samples=5)
        self.previous_centroids = []
        self.time_interval = 0.1  # 10 Hz
        self.current_velocity = 0.0  # Initialize with zero velocity
        self.current_acceleration = 0.0  # Initialize with zero acceleration
        self.current_steering_angle = 0.0  # Initialize with zero steering angle

    def ctrl_cmd_callback(self, msg):
        # Update current velocity, acceleration, and steering angle from /ctrl_cmd topic
        self.current_velocity = msg.linear.x
        self.current_acceleration = msg.linear.y  # Assuming y-axis for forward acceleration
        self.current_steering_angle = msg.angular.z

    def scan_callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:
            return

        pc_xy = self.pc_np[:, :2]
        db = self.dbscan.fit_predict(pc_xy)
        n_cluster = np.max(db) + 1

        current_centroids = []
        cluster_points = []
        for c in range(n_cluster):
            c_tmp = np.mean(pc_xy[db == c, :], axis=0)
            distance = np.sqrt(c_tmp[0]**2 + c_tmp[1]**2)
            angle = np.degrees(np.arctan2(c_tmp[1], c_tmp[0]))
            if distance <= 10 and -30 <= angle <= 30:
                current_centroids.append(c_tmp)
                cluster_points.append([c_tmp[0], c_tmp[1], 1])  # Adding Z coordinate as 1

        self.publish_point_cloud(cluster_points)

        collision_detected = False
        stationary_collision_detected = False
        if self.previous_centroids:
            for i, current_centroid in enumerate(current_centroids):
                if i < len(self.previous_centroids):
                    prev_centroid = self.previous_centroids[i]

                    # Calculate velocity of the cluster
                    cluster_velocity = (current_centroid - prev_centroid) / self.time_interval
                    cluster_speed = np.linalg.norm(cluster_velocity)

                    # Predict future position of the cluster
                    future_cluster_position = current_centroid + cluster_velocity * self.time_interval

                    # Predict future position of the vehicle using kinematic equations
                    future_vehicle_position = self.predict_future_vehicle_position()

                    # Check for potential collision with moving obstacles
                    if self.is_collision(future_cluster_position, future_vehicle_position):
                        rospy.loginfo("Potential collision with moving obstacle detected!")
                        collision_detected = True
                        break

                    # Check for potential collision with stationary obstacles
                    if cluster_speed < 0.1:  # Threshold for stationary obstacle
                        if self.is_collision(current_centroid, future_vehicle_position):
                            rospy.loginfo("Potential collision with stationary obstacle detected!")
                            stationary_collision_detected = True
                            break

        # Publish collision warning (1 for collision, 0 for no collision)
        collision_warning = 1 if collision_detected or stationary_collision_detected else 0
        self.collision_pub.publish(collision_warning)

        # Update previous centroids for next iteration
        self.previous_centroids = current_centroids

    def predict_future_vehicle_position(self):
        # Predict future position of the vehicle using kinematic equations
        # Using bicycle model for vehicle dynamics
        L = 3  # Wheelbase of the vehicle in meters

        # Calculate the turning radius
        if self.current_steering_angle == 0:
            R = float('inf')  # Straight line
        else:
            R = L / np.tan(np.radians(self.current_steering_angle))

        # Calculate the future position of the vehicle
        delta_t = self.time_interval
        future_position_x = self.current_velocity * delta_t
        if R != float('inf'):
            future_position_y = future_position_x / R
            future_position_x = R * np.sin(future_position_y / R)
            future_position_y = R * (1 - np.cos(future_position_y / R))
        else:
            future_position_y = 0.0  # Straight line

        future_position = np.array([future_position_x, future_position_y, 0.0])
        return future_position

    def is_collision(self, future_cluster_position, future_vehicle_position):
        # Simple collision check with a predefined zone around the vehicle's future position
        collision_zone_radius = 1.0
        distance = np.linalg.norm(future_cluster_position[:2] - future_vehicle_position[:2])
        return distance < collision_zone_radius

    def publish_point_cloud(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        # Create PointCloud2 message
        pc2_msg = pc2.create_cloud(header, fields, points)

        # Publish PointCloud2 message
        self.clusterpoints_pub.publish(pc2_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

if __name__ == '__main__':
    LidarCollisionDetector()
    rospy.spin()

