#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class paso_zebra:
    def __init__(self):
        self.img = np.array([])
        self.w = 0
        self.h = 0
        self.bridge = CvBridge()
        self.dt = 0.1
        
        rospy.init_node("paso_zebra")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)

        self.datazebra = rospy.Publisher('/img_processing/zebra',UInt8, queue_size=10)
        self.debug_msg = rospy.Publisher('/ojosZebra',Image,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    def source_callback(self,msg):
        try:
            self.w = msg.width
            self.h = msg.height
            self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except:
            pass

    def timer_callback(self,time):
        negro = np.zeros((5,5),np.uint8)
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("no img zebra")
            pass
        if continuar:
            img_gray = cv2.cvtColor(imagen_resize,cv2.COLOR_BGR2GRAY)
            img_gaus = cv2.GaussianBlur(img_gray,(3,3),cv2.BORDER_DEFAULT)
            imagen_recortada = img_gaus[int(img_gaus.shape[0]/3*2):int(img_gaus.shape[0])-1,int(img_gaus.shape[1]*4/20):int(img_gaus.shape[1]*16/20)-1]
            img_bordes = cv2.Canny(imagen_recortada, 10, 100, apertureSize = 3)
            negro = np.zeros(img_bordes.shape,np.uint8)
            negro = imagen_recortada
            lines = cv2.HoughLinesP(img_bordes,0.1,np.pi/180*1.5,3,100,3)
        
            tamano = []
            continuar = False
            try:
                for x1,y1,x2,y2 in lines[0]:
                    tamano.append(np.sqrt((x1-x2)**2+(y1-y2)**2))
                tamano.sort()
                continuar = True
            except:
                print("No lineas")
            enPasoZebra = False
            if continuar:
                
                if len(lines) > 20:
                    ret,thresh = cv2.threshold(imagen_recortada,70,255,cv2.THRESH_BINARY_INV)
                    #thresh = cv2.dilate(thresh,np.ones((2,2),np.uint8),iterations=1)
                    contours, hirechary = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
                    valCont = 0
                    print("c",len(contours))
                    centros = []
                    for c in contours:
                        if len(c) > 2:
                            xr,yr,wr,hr = cv2.boundingRect(c)
                            cv2.rectangle(negro,(xr,yr),(xr+wr,yr+hr),(255,255,255),1)
                            centros.append(yr-hr/2)
                            valCont += 1
                    sigma = np.std(centros)
                    #negro = thresh
                    print("std_dev",sigma,"buenos",valCont)
                    if sigma < 10 and valCont > 3:
                        enPasoZebra = True
            
            #cv2.putText(negro,"x:" + str(round(self.x,1)) + " y:"+str(round(self.y,1)),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            
            msg_img = Image()
            msg_img = self.bridge.cv2_to_imgmsg(negro)
            self.debug_msg.publish(msg_img)       
        
            if enPasoZebra:
                self.datazebra.publish(1)
            else:
                self.datazebra.publish(0) 

    def stop(self):
        print("shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    zebra = paso_zebra()
    try:
        zebra.run()
    except:
        pass