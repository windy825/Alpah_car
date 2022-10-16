#!/usr/bin/env python
#-*- coding:utf-8 -*-

from math import atan2
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray,Pose
from sklearn.cluster import DBSCAN

# 노드 실행 순서
# 1. DBSCAN Parameter 입력
# 2. 각 Cluster를 대표하는 위치 값 계산
# 3. PointCloud Data로부터 Distance, Angle 값 계산


class SCANCluster:
    def __init__(self):

        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cluster_pub = rospy.Publisher("/clusters", PoseArray, queue_size=15)

        self.pc_np = None

        #TODO: (1) DBSCAN Parameter 입력
 
        # eps = 1.5, min_samples=32
        # eps = 2.0, min_samples=20~40
        # eps = 2.5, min_samples=40
        # eps = 3.0, min_samples=50
        # eps = 3.8, min_samples=35
        # eps = 4.0, min_samples=35
        # eps = 4.5, min_samples=45
        self.dbscan = DBSCAN(eps = 0.5, min_samples=5)

    def callback(self, msg):    
        self.pc_np = self.pointcloud2_to_xyz(msg)
        if len(self.pc_np) == 0:

            cluster_msg = PoseArray()

        else: 
            # 
            pc_xy = self.pc_np[:, :2]
            # print(pc_xy)

            db = self.dbscan.fit_predict(pc_xy)
            # print(db)

            n_cluster = np.max(db) + 1
            # print(n_cluster)

            cluster_msg = PoseArray()

            # cluster_list = []
            cluster_msg.header.frame_id = '/velodyne_points'
            
            for cluster in range(n_cluster):
                cnt = 0
                x_sum = 0
                y_sum = 0
                for idx, label in enumerate(db):
                    if label == cluster:
                        x_sum += pc_xy[idx][0]
                        y_sum += pc_xy[idx][1]
                        cnt += 1


                tmp_pose=Pose()
                tmp_pose.position.x = x_sum / cnt
                tmp_pose.position.y = y_sum / cnt

                cluster_msg.poses.append(tmp_pose)
        # print(cluster_msg)
        self.cluster_pub.publish(cluster_msg)

    def pointcloud2_to_xyz(self, cloud_msg):

        point_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            #TODO: (3) PointCloud Data로부터 Distance, Angle 값 계산

            dist = (point[0]**2 + point[1]**2 + point[2]**2)**0.5
            angle = atan2(point[1], point[0])
            # print(dist, angle)
            
            if point[0] > 0 and 1.50 > point[2] > -1.25 and dist < 50:
                point_list.append((point[0], point[1], point[2], point[3], dist, angle))

        point_np = np.array(point_list, np.float32)

        return point_np



if __name__ == '__main__':

    rospy.init_node('velodyne_clustering', anonymous=True)

    scan_cluster = SCANCluster()

    rospy.spin() 
