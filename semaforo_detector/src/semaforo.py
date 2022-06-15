#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class semaforo_gyr:
    def __init__(self):
        self.img = np.array([])
        self.msk_r = np.array([])
        self.msk_g = np.array([])
        self.msk_y = np.array([])
        self.w = 0
        self.h = 0
        self.ignorarTimer = 38
        self.bridge = CvBridge()
        self.dt = 0.1
        self.red_density = 0
        self.green_density = 0
        self.x = 0
        self.y = 0
        self.gd = 0
        self.rd = 0
        self.last_sent = 4
        self.sem_idx = 0
        rospy.init_node("semaforo_gyr")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        #rospy.Subscriber('/img_properties/green/density',Float32,self.g_callback)
        #rospy.Subscriber('/img_properties/red/density',Float32,self.r_callback)
    
        rospy.Subscriber('/img_processing/sem_toggle',UInt8, self.idx_callback)
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.datasemaforo = rospy.Publisher('/img_processing/sem_Data',UInt8, queue_size=10)
        self.semf_msg = rospy.Publisher('/semforo',Image,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)
    def idx_callback(self,msg):
        self.sem_idx = 1
    def g_callback(self,msg):
        self.gd = msg.data
    def r_callback(self,msg):
        self.rd = msg.data
    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta 
    def source_callback(self,msg):
        try:
            self.w = msg.width
            self.h = msg.height
            self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except:
            pass
    def getDensity(self,color,img):
        imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        if color == 'r':
            red_min = np.array([160,150,20],np.uint8)
            red_max = np.array([179,255,255],np.uint8)
            mask = cv2.inRange(imgHsv,red_min,red_max)
        elif color == 'g':
            green_min = np.array([50,50,50],np.uint8)
            green_max = np.array([90,200,255],np.uint8)
            mask = cv2.inRange(imgHsv,green_min,green_max)
        elif color == 'y':
            yellow_min = np.array([22,93,0],np.uint8)
            yellow_max = np.array([45,255,255],np.uint8)
            mask = cv2.inRange(imgHsv,yellow_min,yellow_max)
        
        mskBGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)

        img_reds = cv2.bitwise_and(img,mskBGR)
        img_grey = cv2.cvtColor(img_reds,cv2.COLOR_BGR2GRAY)
        retval,img_thresh = cv2.threshold(img_grey,20,230,cv2.THRESH_BINARY_INV)
        
        kernel = np.ones((5,5),np.uint8)
        img_erosion = cv2.erode(img_thresh,kernel,iterations=3)
        img_dilate = cv2.dilate(img_erosion,kernel,iterations=3)
        
        #Densidad
        desnsity = 1.0 - np.sum(img_dilate) / float(img_dilate.shape[0] * img_dilate.shape[1]) / 255.0
        return desnsity

    def timer_callback(self,time):
        negro = np.zeros((5,5),np.uint8)
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("la vida es trsiteza")
            pass
        if continuar:
            img_gray = cv2.cvtColor(imagen_resize,cv2.COLOR_BGR2GRAY)

            cv2.putText(negro,str(round(self.rd,2)),(30,35),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            cv2.putText(negro,str(round(self.gd,2)),(30,45),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            if self.sem_idx == 0:
                imgSemf = imagen_resize[int(imagen_resize.shape[0]/12):int(imagen_resize.shape[0]/3),int(imagen_resize.shape[1]*1/10):int(imagen_resize.shape[1]*5/20)]
                print("Sem1")
            else:
                imgSemf = imagen_resize[int(imagen_resize.shape[0]/14):int(imagen_resize.shape[0]*3/12),int(imagen_resize.shape[1]*3/10):int(imagen_resize.shape[1]*6/10)]
                print("Sem2")
            
            imgSemf,color = self.getColour(imgSemf,imagen_resize)
            print("Est")
            msg_img = self.bridge.cv2_to_imgmsg(imgSemf)
            self.semf_msg.publish(msg_img)
            if color != 4 and color != self.last_sent:
                self.datasemaforo.publish(color)
                self.last_sent = color
            cv2.putText(negro,"x:" + str(round(self.x,1)) + " y:"+str(round(self.y,1)),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            

            
            
        

    def getColour(self,imgSemf,imagen_resize):
        imgSef_gray = cv2.cvtColor(imgSemf,cv2.COLOR_BGR2GRAY)
        k = np.ones((5,5),np.uint8)
        ret,thresh = cv2.threshold(imgSef_gray,180,255,cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh,k,iterations=2)
        #thresh = cv2.dilate(thresh,np.ones((2,2),np.uint8),iterations=1)
        contours, hirechary = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        rect = []
        size = []
        #print(imgSemf)
        for c in contours:
            if len(c) > 2:
                xr,yr,wr,hr = cv2.boundingRect(c)
                cv2.rectangle(imgSef_gray,(xr,yr),(xr+wr,yr+hr),(255,255,255),1)
                img_rect = imgSemf[yr:yr+wr,xr:xr+hr,:]
                print(len(imgSemf))
                print(img_rect)
                densities = [self.getDensity('r',img_rect),self.getDensity('y',img_rect),self.getDensity('g',img_rect)]
                rect.append({'x':xr,'y':yr,'w':wr,'h':hr,'md':densities.index(max(densities)),'d':densities[2]})
                size.append(wr*hr)
        color = 4
        cv2.putText(imgSef_gray,"Verde"+str(round(self.getDensity('g',imgSemf),5)),(0,40),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
        if len(rect) > 0:
            print("Solo blob",rect)
            idx = size.index(max(size))
            if rect[idx]['y'] < imgSemf.shape[0]/2 and rect[0]['d'] >=0.2 or self.getDensity('g',imgSemf) >=0.99:
                color = 2
                #self.sem_verde = True
                print("Verde",rect[0])
                cv2.putText(imgSef_gray,"Verde"+str(round(rect[0]['d'],2)),(20,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            else:
                color = 0
                print("Rojo",rect[0])
                cv2.putText(imgSef_gray,"Rojo"+str(rect[0]['y']),(20,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
        return imgSef_gray,color
    def getDensitySmall(self,img):
        return 1 - np.sum(img) / (img.shape[0] * img.shape[1]) / 255
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #publicar
        self.twist_publisher.publish(msg)
        print("Muerte y destruccion o shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    semaforo = semaforo_gyr()
    try:
        semaforo.run()
    except:
        pass