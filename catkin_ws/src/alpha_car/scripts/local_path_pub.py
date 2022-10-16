#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

# 노드 실행 순서 
# 1. Global Path 와 Odometry 데이터 subscriber 생성 
# 2. Local Path publisher 선언
# 3. Local Path 의 Size 결정
# 4. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 5. Global Path 에서 차량 위치와 가장 가까운 포인트(Currenty Waypoint) 탐색
# 6. 가장 가까운 포인트(Currenty Waypoint) 위치부터 Local Path 생성 및 예외 처리 
# 7. Local Path 메세지 Publish


class local_path_pub :
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)
        #TODO: (1) Global Path 와 Odometry 데이터 subscriber 생성 
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber('/global_path', Path, self.global_path_callback)


        #TODO: (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        
        # 초기화
        self.is_odom = False
        self.is_path = False

        #TODO: (3) Local Path 의 Size 결정
        self.local_path_size = 150

        rate = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            if self.is_odom == True and self.is_path == True:
                local_path_msg=Path()
                local_path_msg.header.frame_id='/map'
                
                x=self.x
                y=self.y

                #TODO: (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                min_dis = float('inf')
                current_waypoint = -1
                for idx in range(len(self.global_path_msg.poses)):
                    nx = self.global_path_msg.poses[idx].pose.position.x
                    ny = self.global_path_msg.poses[idx].pose.position.y
                    distance = sqrt((nx - x)**2 + (ny - y)**2)
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = idx


                #TODO: (6) 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1:
                    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        for i in range(current_waypoint, current_waypoint + self.local_path_size):
                            temp_pose = PoseStamped()
                            temp_pose.pose.orientation.w = 1
                            temp_pose.pose.position.x = self.global_path_msg.poses[i].pose.position.x
                            temp_pose.pose.position.y = self.global_path_msg.poses[i].pose.position.y
                            local_path_msg.poses.append(temp_pose)
                    else:
                        for i in range(current_waypoint, len(self.global_path_msg.poses)):
                            temp_pose = PoseStamped()
                            temp_pose.pose.orientation.w = 1
                            temp_pose.pose.position.x = self.global_path_msg.poses[i].pose.position.x
                            temp_pose.pose.position.y = self.global_path_msg.poses[i].pose.position.y
                            local_path_msg.poses.append(temp_pose)
                


                # print(x,y)
                #TODO: (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)


            rate.sleep()

    def odom_callback(self,msg):
        self.is_odom = True
        #TODO: (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def global_path_callback(self,msg):
        self.is_path = True
        self.global_path_msg = msg
        # print(msg.poses[:10])
        # print(msg.poses[-10:])
        # print(msg.poses[-1])

if __name__ == '__main__':
    try:
        test_track=local_path_pub()
    except rospy.ROSInterruptException:
        pass
