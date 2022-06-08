#!/usr/bin/env python

from cv2 import imwrite
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import numpy as np
import cv2
class color_id:
    def __init__(self):
        self.img = np.array([])
        self.w = 0
        self.h = 0
        self.bridge = CvBridge()
        self.dt = 0.1

        rospy.init_node("color_id")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        #rospy.Subscriber('/cam/image_raw',Image,self.source_callback)
        self.red_publisher_msk = rospy.Publisher('/img_properties/red/msk',Image,queue_size=10)
        self.red_publisher_density = rospy.Publisher('/img_properties/red/density',Float32,queue_size=10)
        self.red_publisher_xy = rospy.Publisher('/img_properties/red/xy',Float32,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    def source_callback(self,msg):
        self.w = msg.width
        self.h = msg.height
        self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        #cv2.imwrite('/home/puzzlebot/catkin_ws/src/color_identification/src/oklol.jpg',self.img)
        
    def timer_callback(self, time):
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        red_min = np.array([0,100,20],np.uint8)
        red_max = np.array([10,255,255],np.uint8)
        mask1 = cv2.inRange(imgHsv,red_min,red_max)
        red_min = np.array([160,100,20],np.uint8)
        red_max = np.array([179,255,255],np.uint8)
        mask2 = cv2.inRange(imgHsv,red_min,red_max)
        mask = mask1 + mask2
        #msk_inverse = cv2.bitwise_not(mask)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_reds = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_reds,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        msg_den = Float32()
        desnsity_red = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255
        print(np.sum(img_dilate))
        msg_den.data = desnsity_red
        self.red_publisher_msk.publish(msg_img)
        self.red_publisher_density.publish(msg_den)
        
            
        
    def stop(self):
        print("Muerte y destruccion o shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    colorator = color_id()
    try:
        colorator.run()
    except:
        pass

