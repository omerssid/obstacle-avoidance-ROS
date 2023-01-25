#!/usr/bin/env python3

#import libraries
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from tetra_controller.msg import Engel
from cv_bridge import CvBridge

#This code was used to get red calues as an 8-bit integer 
# and then covert it to HSV to be used in openCV masks 
"""
red = np.uint8([[[0, 255, 255]]])
hsv = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
print(hsv) #[[[ 0 255 255 ]]]
"""

class StartSim():
    def __init__(self):
        global hiz_mesaji
        hiz_mesaji = Twist() #create object of twist type  
        rospy.init_node("Serit_takip")
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.durdur = False
        rospy.Subscriber("camera/rgb/image_raw", Image, self.kameraCallback)
        rospy.Subscriber('/scan', LaserScan, self.lidarCallback)
        rospy.Subscriber('/engel_var', Engel, self.engelCallback)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.engelPub = rospy.Publisher("engel_var", Engel, queue_size=10)
        self.engel_mesaji = Engel()
        rospy.spin()

    def kameraCallback(self, mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj, "bgr8") 
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        alt_red = np.array([0, 100, 100])
        ust_red = np.array([20, 255, 255])
        alt_mavi = np.array([110,50,50])
        ust_mavi = np.array([130,255,255])
        maske = cv2.inRange(hsv, alt_red, ust_red)
        sonuc = cv2.bitwise_and(img, img, mask=maske)
        maske_mavi = cv2.inRange(hsv, alt_mavi, ust_mavi)
        sonuc_mavi = cv2.bitwise_and(img, img, mask=maske_mavi)
        M = cv2.moments(maske)
        Mavi = cv2.moments(maske_mavi)
        h,w,d = img.shape
        self.distance = 1.0
        if self.veri.ranges[0] > self.distance and self.veri.ranges[15] > self.distance and self.veri.ranges[345] > self.distance and not self.durdur: 
            if M['m00'] > 0:
                cv2.hiz_mesaji(img, (int(w/2), int(h/2)), 5, (255, 0, 0), -1)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.hiz_mesaji(img, (cx, cy), 5, (0,0,255), -1)
                sapma = cx - w/2
                hiz_mesaji.linear.x = 0.4
                hiz_mesaji.angular.z =  -sapma/100
                # self.hareket(0.2, -sapma/100)
            else:
                hiz_mesaji.linear.x = 0.0
                hiz_mesaji.angular.z =  0.4
                # self.hareket(h, do)
            rospy.loginfo("Circling") #state situation constantly
        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            hiz_mesaji.linear.x = 0.1 # stop
            hiz_mesaji.angular.z = 0.3 # rotate counter-clockwise
            if self.veri.ranges[0] > self.distance and self.veri.ranges[15] > self.distance and self.veri.ranges[345] > self.distance and self.veri.ranges[45] > self.distance and self.veri.ranges[315] > self.distance:
                #when no any obstacle near detected after rotation
                hiz_mesaji.linear.x = 0.5 #go
                hiz_mesaji.angular.z = 0.1 #rotate
        self.pub.publish(hiz_mesaji) # publish the move object
        cv2.imshow("orijinal", img)
        cv2.imshow("makse", maske)
        cv2.imshow("sonuc", sonuc)
        cv2.waitKey(1)

    
    def lidarCallback(self, veri):
        bolgeler = {        
            'on1':  min(min(veri.ranges[0:19]), 30),
            'on2': min(min(veri.ranges[339:359]), 30),
            'on_sol':  min(min(veri.ranges[10:49]), 30),
            'sol':  min(min(veri.ranges[50:89]), 30),
            'arka':   min(min(veri.ranges[90:268]), 30),
            'sag':   min(min(veri.ranges[269:308]), 30),
            'on_sag':   min(min(veri.ranges[309:348]), 30),
        }

        #Obstacle Avoidance        
        print(bolgeler)
        self.bolgeler= bolgeler
        self.veri = veri
        if self.bolgeler['on1'] and self.bolgeler['on2'] < 1.0: 
            self.engel_mesaji.Engel = True
        else:
            self.engel_mesaji.Engel = False
        self.engelPub.publish(self.engel_mesaji)

    def engelCallback(self, veri):
        self.durdur= veri.Engel
        
    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.pub.publish(Twist())

if __name__ == '__main__':
    try:
        move = StartSim()
	
    except:
        rospy.loginfo("node terminated.")