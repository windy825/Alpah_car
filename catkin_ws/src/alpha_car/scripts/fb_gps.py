#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS
import rospy
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry,Path

# firestore 접근권한 획득
import firebase_admin
from firebase_admin import credentials, firestore
cred = credentials.Certificate("/home/ssafy/catkin_ws/src/serviceAccountkey.json")
firebase_admin.initialize_app(cred)

# firestore 연결
db = firestore.client()
doc_ref = db.collection(u'CurrentLocation').document(u'Ego_0')



class Basic_firebase:
    def __init__(self):
        rospy.init_node('fb_gps', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.callback)
        # rospy.Subscriber("/global_path", Path, self.callback2)
        # rospy.Subscriber("/odom", Odometry, self.callback3)

        self.lon, self.lat = None, None
        self.is_gps = False
        # self.is_me = False
        # self.is_map = False

        # self.x = None
        # self.y = None
        # self.point_xy = []

        self.position_data = {
            u'lati' : None,
            u'long' : None
        }

        number = 1

        rate = rospy.Rate(0.4) #
        while not rospy.is_shutdown():
            if self.is_gps == True:
                db.collection(u'CurrentLocation').document(u'Ego_0').update(self.position_data)
                rate.sleep()

            # if self.is_map == True and self.is_me == True:
            #     answer = 0

            #     for line in self.point_xy:
            #         xx, yy = line.pose.position.x, line.pose.position.y
            #         if ((xx - self.x)**2 + (yy - self.y)**2) < 100:
            #             break
            #         answer += 1
            #     else:
            #         print('no.,..continue!!!!')
            #         continue

            #     print('find!!!!!!!!')
            #     dis_data = {'dis' : (len(self.point_xy) - answer) * 1.3}
            #     db.collection(u'CurrentLocation').document(u'Ego_0').update(dis_data)




    def callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        
        self.position_data = {
            u'lati' : self.lat,
            u'long' : self.lon
        }

        self.is_gps=True

    # def callback2(self, msg):
    #     self.point_xy = msg.poses

    #     self.is_map =True
        
    # def callback3(self, msg):
    #     self.x = msg.pose.pose.position.x
    #     self.y = msg.pose.pose.position.y
    #     self.is_me = True

if __name__ == '__main__':
    try:
        Basic_firebase()
    except rospy.ROSInterruptException:
        pass