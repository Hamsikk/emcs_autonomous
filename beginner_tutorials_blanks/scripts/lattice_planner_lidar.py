#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import rospy
from math import cos, sin, sqrt, pow, atan2
from morai_msgs.msg import CtrlCmd
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray
import numpy as np

class latticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)

        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/ctrl_cmd", CtrlCmd, self.ctrl_cmd_callback)
        rospy.Subscriber("/map_coordinates", Float32MultiArray, self.map_callback)
        rospy.Subscriber("/cluster_global", PoseArray, self.cluster_callback)

        self.lattice_path_pub = rospy.Publisher('/lattice_path', Path, queue_size=1)

        self.is_path = False
        self.is_status = False
        self.is_cluster = False
        self.velocity = 0
        self.current_position = Point()

        self.current_lattice_path = None
        self.last_path_index = 0

        rate = rospy.Rate(30)  # 30hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_status and self.is_cluster:
                if self.current_lattice_path is None or self.reached_goal(self.current_lattice_path):
                    if self.checkObject(self.local_path, self.cluster_data):
                        lattice_path = self.latticePlanner(self.local_path)
                        lattice_path_index = self.collision_check(self.cluster_data, lattice_path)

                        rospy.loginfo(f"Selected lattice path index: {lattice_path_index}")

                        self.current_lattice_path = lattice_path[lattice_path_index]
                    else:
                        self.current_lattice_path = None  # lattice path를 초기화하여 local path로 전환

                if self.current_lattice_path is not None:
                    self.lattice_path_pub.publish(self.current_lattice_path)
                else:
                    self.lattice_path_pub.publish(self.local_path)
            rate.sleep()

    def reached_goal(self, path):
        if not path.poses:
            return False

        goal_position = path.poses[-1].pose.position

        distance_to_goal = sqrt(pow(self.current_position.x - goal_position.x, 2) +
                                pow(self.current_position.y - goal_position.y, 2))

        if distance_to_goal < 5:  # 차량이 목표 지점에 m 이내로 접근하면 경로 변경
            rospy.loginfo("Reached the goal of the lattice path.")
            self.current_lattice_path = None  # lattice path를 초기화
            return True
        return False

    def checkObject(self, ref_path, cluster_data):
        is_crash = False
        for obstacle in cluster_data.poses:
            for path in ref_path.poses:
                dis = sqrt(pow(path.pose.position.x - obstacle.position.x, 2) +
                           pow(path.pose.position.y - obstacle.position.y, 2))
                if dis < 2.35:
                    is_crash = True
                    break
        return is_crash

    def collision_check(self, cluster_data, out_path):
        selected_lane = -1
        lane_weight = [3, 3, 3, 3, 2, 1]  # reference path

        for obstacle in cluster_data.poses:
            for path_num in range(len(out_path)):
                for path_pos in out_path[path_num].poses:
                    dis = sqrt(pow(obstacle.position.x - path_pos.pose.position.x, 2) +
                               pow(obstacle.position.y - path_pos.pose.position.y, 2))
                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))
        return selected_lane

    def path_callback(self, msg):
        self.is_path = True
        self.local_path = msg

    def ctrl_cmd_callback(self, ctrl_cmd_msg):
        self.velocity = ctrl_cmd_msg.velocity

    def map_callback(self, map_msg):
        self.current_position.x = map_msg.data[0]
        self.current_position.y = map_msg.data[1]
        self.is_status = True

    def cluster_callback(self, msg):
        self.is_cluster = True
        self.cluster_data = msg

    def latticePlanner(self, ref_path):
        out_path = []
        vehicle_pose_x = self.current_position.x
        vehicle_pose_y = self.current_position.y
        vehicle_velocity = self.velocity * 3.6

        look_distance = int(vehicle_velocity * 0.5 * 2)

        if look_distance < 20:
            look_distance = 20

        if len(ref_path.poses) > look_distance:
            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance * 2].pose.position.x, ref_path.poses[look_distance * 2].pose.position.y)

            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            trans_matrix = np.array([
                [cos(theta), -sin(theta), translation[0]],
                [sin(theta), cos(theta), translation[1]],
                [0, 0, 1]
            ])

            det_trans_matrix = np.array([
                [trans_matrix[0][0], trans_matrix[1][0], -(trans_matrix[0][0] * translation[0] + trans_matrix[1][0] * translation[1])],
                [trans_matrix[0][1], trans_matrix[1][1], -(trans_matrix[0][1] * translation[0] + trans_matrix[1][1] * translation[1])],
                [0, 0, 1]
            ])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_trans_matrix.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_trans_matrix.dot(world_ego_vehicle_position)
            lane_off_set = [-4, -1.75, -1, 1, 1.75, 4.0]
            local_lattice_points = []

            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])

            for end_point in local_lattice_points:
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0] * 1.1
                ps = local_ego_vehicle_position[1][0]
                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs, int(x_num)):
                    x.append(i * x_interval)

                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0, len(y)):
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = trans_matrix.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            # Add_point
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses))

            for i in range(look_distance * 2, add_point_size):
                if i + 1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y, ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)
                    tmp_translation = [ref_path.poses[i].pose.position.x, ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([
                        [cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                        [sin(tmp_theta), cos(tmp_translation[1])],
                        [0, 0, 1]
                    ])

                    for lane_num in range(len(lane_off_set)):
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)

            for i in range(len(out_path)):
                globals()['lattice_pub_{}'.format(i + 1)] = rospy.Publisher('/lattice_path_{}'.format(i + 1), Path, queue_size=1)
                globals()['lattice_pub_{}'.format(i + 1)].publish(out_path[i])

        return out_path

if __name__ == '__main__':
    try:
        latticePlanner()
    except rospy.ROSInterruptException:
        pass

