#!/usr/bin/env python

from cv2 import imwrite
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
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
        self.orange_publisher_msk = rospy.Publisher('/img_properties/orange/msk',Image,queue_size=10)
        self.orange_publisher_density = rospy.Publisher('/img_properties/orange/density',Float32,queue_size=10)
        self.orange_publisher_xy = rospy.Publisher('/img_properties/orange/xy',Float32MultiArray,queue_size=10)
        self.purple_publisher_msk = rospy.Publisher('/img_properties/purple/msk',Image,queue_size=10)
        self.purple_publisher_density = rospy.Publisher('/img_properties/purple/density',Float32,queue_size=10)
        self.purple_publisher_xy = rospy.Publisher('/img_properties/purple/xy',Float32MultiArray,queue_size=10)
        self.pink_publisher_msk = rospy.Publisher('/img_properties/pink/msk',Image,queue_size=10)
        self.pink_publisher_density = rospy.Publisher('/img_properties/pink/density',Float32,queue_size=10)
        self.pink_publisher_xy = rospy.Publisher('/img_properties/pink/xy',Float32MultiArray,queue_size=10)
        self.blueF_publisher_msk = rospy.Publisher('/img_properties/blueF/msk',Image,queue_size=10)
        self.blueF_publisher_density = rospy.Publisher('/img_properties/blueF/density',Float32,queue_size=10)
        self.blueF_publisher_xy = rospy.Publisher('/img_properties/blueF/xy',Float32MultiArray,queue_size=10)
        self.cielo_publisher_msk = rospy.Publisher('/img_properties/cielo/msk',Image,queue_size=10)
        self.cielo_publisher_density = rospy.Publisher('/img_properties/cielo/density',Float32,queue_size=10)
        self.cielo_publisher_xy = rospy.Publisher('/img_properties/cielo/xy',Float32MultiArray,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback_orange)
        self.t2 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback_purple)
        self.t3 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback_pink)
        self.t4 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback_blueF)
        self.t5 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback_cielo)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    def source_callback(self,msg):
        self.w = msg.width
        self.h = msg.height
        self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        #cv2.imwrite('/home/puzzlebot/catkin_ws/src/color_identification/src/oklol.jpg',self.img)
        
    def timer_callback_orange(self, time):
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
        orange_min = np.array([12,67,61],np.uint8) # 17.2 72.2 71.8
        orange_max = np.array([22,82,81],np.uint8)
        mask = cv2.inRange(imgHsv,orange_min,orange_max)
        #msk_inverse = cv2.bitwise_not(mask)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_oranges = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_oranges,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity_orange = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255

        #CM
        img_cm = cv2.subtract(np.ones((self.h,self.w),np.uint8)*255,img_dilate)
        rx = np.arange(0,self.w)
        cmx = 0
        mass = np.sum(img_cm)
        for row in img_cm:
            cmx += np.sum(np.multiply(rx,row))
        cmx /= mass

        cmy = 0
        ry = np.arange(0,self.h)
        img_trans = np.transpose(img_cm)
        for col in img_trans:
            cmy += np.sum(np.multiply(ry,col))
        cmy /= mass
        #Enviar Imagen
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        self.orange_publisher_msk.publish(msg_img)
        #Enviar densidad
        msg_den = Float32()
        msg_den.data = desnsity_orange
        self.orange_publisher_density.publish(msg_den)
        #Enviar CM
        msg_cm = Float32MultiArray()
        msg_cm.data = [cmx, cmy]
        self.orange_publisher_xy.publish(msg_cm)
    def timer_callback_purple(self,time):
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV) #264.6 32.9 54.1
        purple_min = np.array([259,28,44],np.uint8)
        purple_max = np.array([27,38,64],np.uint8)
        mask = cv2.inRange(imgHsv,purple_min,purple_max)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_purples = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_purples,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity_purple = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255

        #CM
        img_cm = cv2.subtract(np.ones((self.h,self.w),np.uint8)*255,img_dilate)
        rx = np.arange(0,self.w)
        cmx = 0
        mass = np.sum(img_cm)
        for row in img_cm:
            cmx += np.sum(np.multiply(rx,row))
        cmx /= mass

        cmy = 0
        ry = np.arange(0,self.h)
        img_trans = np.transpose(img_cm)
        for col in img_trans:
            cmy += np.sum(np.multiply(ry,col))
        cmy /= mass
        #Enviar Imagen
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        self.purple_publisher_msk.publish(msg_img)
        #Enviar densidad
        msg_den = Float32()
        msg_den.data = desnsity_purple
        self.purple_publisher_density.publish(msg_den)
        #Enviar CM
        msg_cm = Float32MultiArray()
        msg_cm.data = [cmx, cmy]
        self.purple_publisher_xy.publish(msg_cm)
    def timer_callback_pink(self,time):
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV) # 358.4 55 71.7
        pink_min = np.array([354,50,62],np.uint8)
        pink_max = np.array([363,60,82],np.uint8)
        mask = cv2.inRange(imgHsv,pink_min,pink_max)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_pinks = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_pinks,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity_pink = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255

        #CM
        img_cm = cv2.subtract(np.ones((self.h,self.w),np.uint8)*255,img_dilate)
        rx = np.arange(0,self.w)
        cmx = 0
        mass = np.sum(img_cm)
        for row in img_cm:
            cmx += np.sum(np.multiply(rx,row))
        cmx /= mass

        cmy = 0
        ry = np.arange(0,self.h)
        img_trans = np.transpose(img_cm)
        for col in img_trans:
            cmy += np.sum(np.multiply(ry,col))
        cmy /= mass
        #Enviar Imagen
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        self.pink_publisher_msk.publish(msg_img)
        #Enviar densidad
        msg_den = Float32()
        msg_den.data = desnsity_pink
        self.pink_publisher_density.publish(msg_den)
        #Enviar CM
        msg_cm = Float32MultiArray()
        msg_cm.data = [cmx, cmy]
        self.pink_publisher_xy.publish(msg_cm)
    def timer_callback_blueF(self,time):   
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV) # 214.8 70 50.1
        blueF_min = np.array([210,65,40],np.uint8)
        blueF_max = np.array([220,75,60],np.uint8)
        mask = cv2.inRange(imgHsv,blueF_min,blueF_max)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_blueFs = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_blueFs,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity_blueF = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255

        #CM
        img_cm = cv2.subtract(np.ones((self.h,self.w),np.uint8)*255,img_dilate)
        rx = np.arange(0,self.w)
        cmx = 0
        mass = np.sum(img_cm)
        for row in img_cm:
            cmx += np.sum(np.multiply(rx,row))
        cmx /= mass

        cmy = 0
        ry = np.arange(0,self.h)
        img_trans = np.transpose(img_cm)
        for col in img_trans:
            cmy += np.sum(np.multiply(ry,col))
        cmy /= mass
        #Enviar Imagen
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        self.blueF_publisher_msk.publish(msg_img)
        #Enviar densidad
        msg_den = Float32()
        msg_den.data = desnsity_blueF
        self.blueF_publisher_density.publish(msg_den)
        #Enviar CM
        msg_cm = Float32MultiArray()
        msg_cm.data = [cmx, cmy]
        self.blueF_publisher_xy.publish(msg_cm)
    def timer_callback_cielo(self,time):
        imgHsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV) # 196.8 93.8 77.1
        cielo_min = np.array([192,89,67],np.uint8)
        cielo_max = np.array([202,99,87],np.uint8)
        mask = cv2.inRange(imgHsv,cielo_min,cielo_max)
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_cielos = cv2.bitwise_and(self.img,mskBGR)
        img_grey = cv2.cvtColor(img_cielos,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity_cielo = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255

        #CM
        img_cm = cv2.subtract(np.ones((self.h,self.w),np.uint8)*255,img_dilate)
        rx = np.arange(0,self.w)
        cmx = 0
        mass = np.sum(img_cm)
        for row in img_cm:
            cmx += np.sum(np.multiply(rx,row))
        cmx /= mass

        cmy = 0
        ry = np.arange(0,self.h)
        img_trans = np.transpose(img_cm)
        for col in img_trans:
            cmy += np.sum(np.multiply(ry,col))
        cmy /= mass
        #Enviar Imagen
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(img_dilate)
        self.cielo_publisher_msk.publish(msg_img)
        #Enviar densidad
        msg_den = Float32()
        msg_den.data = desnsity_cielo
        self.cielo_publisher_density.publish(msg_den)
        #Enviar CM
        msg_cm = Float32MultiArray()
        msg_cm.data = [cmx, cmy]
        self.cielo_publisher_xy.publish(msg_cm)

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

