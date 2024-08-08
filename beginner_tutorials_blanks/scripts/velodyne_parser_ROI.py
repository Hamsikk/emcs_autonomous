#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32

class SCANParser:

    def __init__(self):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.dist_pub = rospy.Publisher("dist_forward", Float32, queue_size=10)
        self.roi_pub = rospy.Publisher("/roi_points", PointCloud2, queue_size=10)
        self.pc_np = None

        # Define ROI boundaries (example values, modify as needed)
        self.x_min = 0.0
        self.x_max = 25.0
        self.y_min = -2.3
        self.y_max = 4.0
        self.z_min = -0.3
        self.z_max = 0.7

        # Define height threshold for tree removal (example value, modify as needed)
        self.tree_height_threshold = 1.0

    def callback(self, msg):
        self.pc_np = self.pointcloud2_to_xyz(msg)
        self.pc_np = self.remove_trees(self.pc_np)
        d_min = self.calc_dist_forward()
        dist_msg = Float32()
        dist_msg.data = d_min
        self.dist_pub.publish(dist_msg)

        # Publish the ROI filtered PointCloud2
        roi_msg = self.xyz_to_pointcloud2(self.filtered_points[:, :3])  # Pass only x, y, z
        self.roi_pub.publish(roi_msg)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            dist = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            angle = np.arctan2(point[1], point[0])
            if point[0] > 0 and point[2] > -1.3 and dist < 50:
                point_list.append((point[0], point[1], point[2], dist, angle))

        point_np = np.array(point_list, np.float32)
        return point_np

    def remove_trees(self, points):
        # Apply height threshold to remove trees
        mask = points[:, 2] < self.tree_height_threshold
        return points[mask]

    def calc_dist_forward(self):
        # Apply ROI filter
        roi_bool = (self.pc_np[:, 0] >= self.x_min) & (self.pc_np[:, 0] <= self.x_max) & \
                   (self.pc_np[:, 1] >= self.y_min) & (self.pc_np[:, 1] <= self.y_max) & \
                   (self.pc_np[:, 2] >= self.z_min) & (self.pc_np[:, 2] <= self.z_max)

        self.filtered_points = self.pc_np[roi_bool]
        
        if len(self.filtered_points) == 0:
            return float('inf')  # No points in ROI

        d_min = np.min(self.filtered_points[:, 3])
        return d_min

    def xyz_to_pointcloud2(self, points):
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "velodyne"
        
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        
        return pc2.create_cloud(header, fields, points)

if __name__ == '__main__':
    rospy.init_node('velodyne_parser', anonymous=True)
    scan_parser = SCANParser()
    rospy.spin()

