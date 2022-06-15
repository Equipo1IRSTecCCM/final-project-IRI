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


class line_follower:
    def __init__(self):
        self.img = np.array([])
        self.ignorarTimer = 38
        self.bridge = CvBridge()
        self.dt = 0.1
        self.max_v = 0.3
        self.estancado = False
        self.red_density = 0
        self.green_density = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.antDist = []
        self.doing_pp = False
        self.sem_verde = False
        self.max_w = 0.15
        self.enRecta = False
        self.gd = 0
        self.rd = 0
        
        rospy.init_node("line_follower")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        #rospy.Subscriber('/pp/finish',UInt8,self.finsh_pp_callback)
        rospy.Subscriber('/img_properties/green/density',Float32,self.g_callback)
        rospy.Subscriber('/img_properties/red/density',Float32,self.r_callback)

        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.twist_publisher = rospy.Publisher('/img_processing/edge_cmd_vel', Twist, queue_size=10)
        self.debug_msg = rospy.Publisher('/img_processing/edgeOjos',Image,queue_size=10)
        #self.points_publisher = rospy.Publisher('/pp/points', Float32MultiArray, queue_size=10)
        #self.pp_init = rospy.Publisher('/pp/init', UInt8, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)
        
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

    def getDistance(self,x1,y1,x2,y2):
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    def getAngle(self,x1,y1,x2,y2):
        return np.arctan2(y1-y2,x1-x2)
    
    def skeletonize(self,img):
        size = np.size(img)
        skel = np.zeros(img.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        while True:
            #Step 2: Open the image
            open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
            #Step 3: Substract open from the original image
            temp = cv2.subtract(img, open)
            #Step 4: Erode the original image and refine the skeleton
            eroded = cv2.erode(img, element)
            skel = cv2.bitwise_or(skel,temp)
            img = eroded.copy()
            # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
            if cv2.countNonZero(img)==0:
                break
        return skel
    def timer_callback(self,time):
        omega = 0
        vel = 0.1
        negro = np.zeros((5,5),np.uint8)
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("la vida es trsiteza")
            pass
        if continuar:
            img = cv2.cvtColor(imagen_resize,cv2.COLOR_BGR2GRAY)
            img_gaus = cv2.GaussianBlur(img,(3,3),cv2.BORDER_DEFAULT)
            #img_gaus = cv2.GaussianBlur(img_gaus,(3,3),cv2.BORDER_DEFAULT)
            img_g = img_gaus[int(img_gaus.shape[0]*2/6):int(img_gaus.shape[0]*4/6),int(img_gaus.shape[1]*17/20):int(img_gaus.shape[1])-1]
            _,img = cv2.threshold(img_g, 80, 255, cv2.THRESH_BINARY_INV)

            skel = self.skeletonize(img)
            lines = cv2.HoughLinesP(skel,0.1,np.pi/180*1.5,3,minLineLength=5,maxLineGap=100)
            negro = np.zeros(skel.shape,np.uint8)
            tamano = []
            continuar = False
            try:
                for x1,y1,x2,y2 in lines[0]:
                    tamano.append(np.sqrt((x1-x2)**2+(y1-y2)**2))
                tamano.sort()
                continuar = True
            except:
                print("No lineas")
            
            if continuar:
                '''
                puntoMedio = [int(negro.shape[1]/2),int(negro.shape[0]-3)]
                distancias = [self.getDistance(x1,y1,puntoMedio[0],puntoMedio[1]),self.getDistance(x2,y2,puntoMedio[0],puntoMedio[1])]
            
                print("Puntomedio y distancias",puntoMedio,distancias)
                if distancias[0] > distancias[1]:
                    puntoObjetivo = [x1,y1]
                else:
                    puntoObjetivo = [x2,y2]
                if x1 == x2:
                    pendiente = np.sign(float(y1-y2)) / 0.0001
                else:
                    pendiente = float(y1-y2)/float(x1-x2)
                pendiente_obj = 1.0
                #cv2.putText(skel,str(round(pendiente,2)),(30,35),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                d = pendiente - pendiente_obj

                d /= pendiente

                print("Drol",d)
                dtheta = d * 0.12
                '''
                idx = tamano.index(max(tamano))
                x1,y1,x2,y2 = lines[0][idx]
                cv2.line(negro,(x1,y1),(x2,y2),(255,255,255),1)
                if x1 == x2:
                    pendiente = 10
                else:
                    pendiente = float(y1-y2)/float(x1-x2)
                if pendiente > 10:
                    pendiente = 10
                cv2.putText(skel,str(round(pendiente,1)) ,(10,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                dtheta = 0
                
                cv2.putText(skel,str(x1)+" "+str(x2),(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                cv2.putText(skel,str(y1)+" "+str(y2),(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)


                
                
                msg = Twist()
                msg.linear.x = vel
                msg.linear.y = 0
                msg.linear.z = 0
                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = dtheta
                if abs(msg.angular.z) > self.max_w:
                    msg.angular.z = self.max_w * np.sign(msg.angular.z)
                cv2.putText(skel,str(round(dtheta,3)),(30,40),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                #publicar
                
                #self.twist_publisher.publish(msg)

            
            #cv2.putText(negro,"x:" + str(round(self.x,1)) + " y:"+str(round(self.y,1)),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            
            msg_img = Image()
            msg_img = self.bridge.cv2_to_imgmsg(negro)
            self.debug_msg.publish(msg_img)

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
    lineator = line_follower()
    try:
        lineator.run()
    except:
        pass
