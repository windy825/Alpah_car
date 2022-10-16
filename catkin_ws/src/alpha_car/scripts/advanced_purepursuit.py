#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd,EgoVehicleStatus,GetTrafficLightStatus
from alpha_car.msg import global_data
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        rospy.Subscriber(local_path_name, Path, self.path_callback)

        
        rospy.Subscriber("/global_path", Path, self.global_path_callback)
        rospy.Subscriber("/global_data", global_data, self.global_data_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)


        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.is_path = False
        self.is_odom = False 
        self.is_status = False
        self.is_global_path = False
        self.is_get_traffic = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        self.vehicle_length = 2.6
        self.lfd = 10
        self.min_lfd = 10
        self.max_lfd = 30
        self.lfd_gain = 0.78
        self.target_velocity = 40

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)
        self.velocity_list = None

        while True:
            if self.is_global_path == True:
                break


        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path == True and self.is_odom == True and self.is_status == True and self.is_global_path == True and self.velocity_list:
                prev_time = time.time()
                
                self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                
                self.target_velocity = self.velocity_list[self.current_waypoint]*3.6
                

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering = steering
                else : 
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0
                
                output = self.pid.pid(self.target_velocity,self.status_msg.velocity.x*3.6)
                print(output)

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output
                    
                print(len(self.path.poses))
                if len(self.path.poses) <= 20 and self.ctrl_cmd_msg.brake < 1:
                    self.ctrl_cmd_msg.brake = 10


                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  

    def odom_callback(self,msg):
        self.is_odom=True
        odom_quaternion=(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw=euler_from_quaternion(odom_quaternion)
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y

    def status_callback(self,msg): 
        self.is_status=True
        self.status_msg=msg    
        
    def global_path_callback(self,msg):
        self.global_path = msg
        self.is_global_path = False

        # 임시적으로 에러 해결
        self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
        self.is_global_path = True

    def global_data_callback(self, msg):
        self.global_data = msg
        self.is_global_data = True
        print(msg)

    
    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self,):
        self.lfd = self.lfd_gain * min(self.max_lfd, max(self.min_lfd, self.status_msg.velocity.x))
        rospy.loginfo(self.lfd)
        
        
        vehicle_position=self.current_postion
        self.is_look_forward_point= False

        translation = [vehicle_position.x, vehicle_position.y]
        trans_matrix = np.array([[cos(self.vehicle_yaw), -sin(self.vehicle_yaw), 0],
                                 [sin(self.vehicle_yaw),  cos(self.vehicle_yaw), 0],
                                 [0                    , 0                     , 1]])

        det_trans_matrix = np.linalg.inv(trans_matrix)

        for idx, pose in enumerate(self.path.poses) :
            path_point = pose.pose.position
            
            global_path_point = [path_point.x - translation[0], path_point.y - translation[1], 1]
            local_path_point = det_trans_matrix.dot(global_path_point)    

            # 후진하지 않기 위해서 
            if local_path_point[0] > 0 :
                dis = (local_path_point[0]**2 + local_path_point[1]**2)**0.5

                if dis >= self.lfd :
                    self.forward_point = local_path_point
                    self.is_look_forward_point = True
                    break
        
        if self.is_look_forward_point:
            theta = atan2(self.forward_point[1], self.forward_point[0])
        else:
            theta = 0
            
        steering = atan2(2*self.vehicle_length*sin(theta), self.lfd)


        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel



        p_control = self.p_gain*error
        self.i_control += error*self.controlTime    # 에러 값들의 총합
        d_control = self.d_gain*(error-self.prev_error)/self.controlTime

        output = p_control + self.i_gain*self.i_control + d_control
        self.prev_error = error

        return output

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))



            A = np.array(x_list)
            B = np.array(y_list)
            a, b, c = np.dot(np.linalg.pinv(A), B)
            
            r = (a**2 + b**2 - c)**0.5


            v_max = (r * 9.8 * 0.8) ** 0.5


            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(20)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
