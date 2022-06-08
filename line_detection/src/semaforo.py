#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class line_follower:
    def __init__(self):
        self.img = np.array([])
        self.w = 0
        self.h = 0
        self.ignorarTimer = 30
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
        rospy.init_node("line_follower")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        
        
        self.debug_msg = rospy.Publisher('/semforo',Image,queue_size=10)

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
        try:
            imagen_recortada = self.img[0:int(self.img.shape[0]/2),0:int(self.img.shape[1]/4)]
            
        except:
            imagen_recortada = np.ones((70,240),np.uint8)
        msg_img = Image()
        msg_img = self.bridge.cv2_to_imgmsg(imagen_recortada)
        self.debug_msg.publish(msg_img)
            
        
        

        

    def stop(self):
        print("Semaforo Muerte y destruccion o shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lineator = line_follower()
    try:
        lineator.run()
    except:
        pass