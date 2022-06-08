#!/usr/bin/env python
#<node name="semaforo_gyr" pkg="semaforoChido" type="semaforo.py"/>
from turtle import pen
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
        
        rospy.init_node("semaforo_gyr")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        rospy.Subscriber('/img_properties/green/density',Float32,self.g_callback)
        rospy.Subscriber('/img_properties/red/density',Float32,self.r_callback)
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.datasemaforo = rospy.Publisher('/img_processing/sem_Data',UInt8, queue_size=10)
        self.semf_msg = rospy.Publisher('/semforo',Image,queue_size=10)
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
    def getDensity(self,color,img):
        imgHsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        if color == 'r':
            red_min = np.array([0,100,20],np.uint8)
            red_max = np.array([10,255,255],np.uint8)
            mask1 = cv2.inRange(imgHsv,red_min,red_max)
            red_min = np.array([160,100,20],np.uint8)
            red_max = np.array([179,255,255],np.uint8)
            mask2 = cv2.inRange(imgHsv,red_min,red_max)
            mask = mask1 + mask2
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
        desnsity = 1 - np.sum(img_dilate) / (img_dilate.shape[0] * img_dilate.shape[1]) / 255
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
            img_gaus = cv2.GaussianBlur(img_gray,(3,3),cv2.BORDER_DEFAULT)
            if self.ignorarTimer < 35:
                imagen_recortada = img_gaus[int(img_gaus.shape[0]/3*2):int(img_gaus.shape[0])-1,int(img_gaus.shape[1]*4/5):int(img_gaus.shape[1])-1]
                self.ignorarTimer +=1

            img_bordes = cv2.Canny(imagen_recortada, 10, 100, apertureSize = 3)
            k = np.ones((3,3),np.uint8)

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
            if continuar:
                '''
                if self.rd > 0.15:
                    self.estancado = True
                if self.gd > 0.15:
                    self.estancado = False
                '''
                cv2.putText(negro,str(round(self.rd,2)),(30,35),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                cv2.putText(negro,str(round(self.gd,2)),(30,45),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
                if self.y < 0.15:
                    imgSemf = img_gray[int(img_gray.shape[0]/12):int(img_gray.shape[0]/3),0:int(img_gray.shape[1]*3/10)]
                    print("Sem1")
                else:
                    imgSemf = img_gray[int(img_gray.shape[0]/14):int(img_gray.shape[0]*3/12),int(img_gray.shape[1]*3/10):int(img_gray.shape[1]*6/10)]
                    print("Sem2")
                if self.estancado:
                    imgSemf,color = self.getColour(imgSemf,imagen_resize)
                    print("Est")
                msg_img = self.bridge.cv2_to_imgmsg(imgSemf)
                self.semf_msg.publish(msg_img)
                self.datasemaforo.publish(color)
                    
            cv2.putText(negro,"x:" + str(round(self.x,1)) + " y:"+str(round(self.y,1)),(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            msg_img = Image()
            msg_img = self.bridge.cv2_to_imgmsg(negro)
            self.debug_msg.publish(msg_img)

            
            
        

    def getColour(self,imgSemf,imagen_resize):
        kernel = np.ones(imgSemf.shape,np.uint8)
        imgSemf = cv2.multiply(imgSemf,kernel)
        k = np.ones((5,5),np.uint8)
        ret,thresh = cv2.threshold(imgSemf,180,255,cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh,k,iterations=2)
        #thresh = cv2.dilate(thresh,np.ones((2,2),np.uint8),iterations=1)
        contours, hirechary = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        rect = []
    
        for c in contours:
            if len(c) > 2:
                xr,yr,wr,hr = cv2.boundingRect(c)
                rect.append({'x':xr,'y':yr,'w':wr,'h':hr})
                cv2.rectangle(imgSemf,(xr,yr),(xr+wr,yr+hr),(255,255,255),1)
        if len(rect) == 1:
            print("Solo blob",rect)
            if rect[0]['y'] < imgSemf.shape[0]/2:
                color = 2
                #self.sem_verde = True
                print("Verde",rect[0])
                cv2.putText(imgSemf,"Verde"+str(rect[0]['y']),(20,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            else:
                color = 0
                print("Rojo",rect[0])
                cv2.putText(imgSemf,"Rojo"+str(rect[0]['y']),(20,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
        '''
        #msk_r = cv2.resize(self.msk_r,None,fx=0.3,fy=0.3)
        #msk_g = cv2.resize(self.msk_g,None,fx=0.3,fy=0.3)
        #msk_y = cv2.resize(self.msk_y,None,fx=0.3,fy=0.3)
        for r in rect:
            densities = []
        
            #msk_img = msk_r[r['x']:r['x']+r['w'],r['y']:r['y']+r['h']]
            #densities.append(self.getDensitySmall(msk_img))
            #msk_img = msk_g[r['x']:r['x']+r['w'],r['y']:r['y']+r['h']]
            #densities.append(self.getDensitySmall(msk_img))
            #msk_img = msk_y[r['x']:r['x']+r['w'],r['y']:r['y']+r['h']]
            #densities.append(self.getDensitySmall(msk_img))
            #img = imagen_resize[r['x']:r['x']+r['w'],r['y']:r['y']+r['h']]
            #densities = [self.getDensity('r',img),self.getDensity('g',img),self.getDensity('y',img)]
            print(r,densities)
            
            
            maximos.append([densities.index(max(densities)),max(densities)])
            cv2.putText(imgSemf,str(round(densities[0],3)),(r['x'],r['y']),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            cv2.putText(imgSemf,str(round(densities[1],3)),(r['x'],r['y']+8),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
            cv2.putText(imgSemf,str(round(densities[2],3)),(r['x'],r['y']+16),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
        
        
        maxi = 0
        idx_max = 1
        for index,den in maximos:
            if den > maxi:
                maxi = den
                idx_max = index
        if idx_max == 0:
            colour = "rojo"
        elif idx_max == 1:
            colour = "verde"
        else:
            colour = "amarillo"
        cv2.putText(imgSemf,colour,(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1)
        '''
        return imgSemf,color
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