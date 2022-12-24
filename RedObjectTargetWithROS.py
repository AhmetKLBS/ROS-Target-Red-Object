#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class robot_camera():
    def __init__(self):
        rospy.init_node("kamera")
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_cb)
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        self.SpeedMessage= Twist()
        self.bridge = CvBridge()
        rospy.spin()
        rospy.init_node("LaserScanData")
        self.pub=rospy.Publisher("Cmd_Value",Twist,queue_size=10)
        self.speedMessage=Twist()
        rospy.Subscriber("Scan", LaserScan,self.laserCallback)
    def lazerCallback(self,mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        sol = list(mesaj.ranges[80:100])
        sag = list(mesaj.ranges[260:280])
        arka = list(mesaj.ranges[170:190])
        min_on = min(on)
        min_sol = min(sol)
        min_sag = min(sag)
        min_arka = min(arka)
        print(min_on,min_sol,min_sag,min_arka)
        if min_on < 1.0:
            self.speedMessage.linear.x=0.0
            self.speedMessage.linear.y=0.0
    def camera_cb(self, mesaj):
        self.cap = self.bridge.imgmsg_to_cv2(mesaj, "bgr8") #Bu komut satırı image mesajını bir opencv 'cv::Mat tipine dönüştürür.
        
        # These lines of code do red area detection (Bu kod satırları kırmızı alan tespiti yapar)
        #lower_red, upper_red komutları
        lower_red = np.array([0,50,50]) 
        upper_red = np.array([10,255,255])
        
        hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)
    
        mask = cv2.inRange(hsv, lower_red, upper_red)
        mask = cv2.erode(mask, (5, 5), iterations=9)
        mask = cv2.medianBlur(mask, 7)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))
        mask = cv2.dilate(mask, (5, 5), iterations=1)

        _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    
        cnts,_ = cv2.findContours(thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

        frame_merkez_x = self.cap.shape[1]/2
        frame_merkez_y = self.cap.shape[0]/2

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            cv2.circle(self.cap, (int(x), int(y)),int(radius),(0, 0, 255), 2)
            cv2.putText(self.cap, "X : " + str(round(x,2)), (int(x)+int(radius)+5,int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(self.cap, "Y : " + str(round(y,2)), (int(x)+int(radius)+5,int(y)+35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.line(self.cap, (int(frame_merkez_x),int(frame_merkez_y)),(int(x),int(y)),(0,0,0),3)
        #These lines of code li Alignment tracking (Bu kod  satırları hizalama takip etme)

            hata_x = int(x) - int(frame_merkez_x)
            
            self.SpeedMessage.linear.x = 0.2 # ileri gitmesi icin
            self.SpeedMessage.angular.z = -hata_x/100
            self.pub.publish(self.SpeedMessage)
            if radius > 150:
                self.SpeedMessage.linear.x = 0.0
                self.SpeedMessage.angular.z = 0.0
                self.pub.publish(self.SpeedMessage) 

        else:
            #self.SpeedMessage.linear.x = 0.0
            self.SpeedMessage.angular.z = 0.5
            self.pub.publish(self.SpeedMessage)
        cv2.imshow("frame", self.cap)
        cv2.waitKey(1)

robot_camera()
