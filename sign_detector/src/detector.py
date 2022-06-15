#!/usr/bin/env python 
 
import rospy 
from std_msgs.msg import String 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge 
import numpy as np 
import cv2 
from std_msgs.msg import UInt8
import math 
 
class Signal_Detector: 
    def __init__(self): 
        self.activate = False 
        self.bridge = CvBridge() 
        self.image_raw = None 
        self.dt = 0.05
        self.cut_y = (int(3*720.0/4.0),720) 
        self.cut_x = (0,1280) 
        self.tem = [] 
 
        self.tml = 5 
        self.orb = cv2.ORB_create(500) 
        self.orb2 = cv2.ORB_create(1000) 
        self.flann = cv2.FlannBasedMatcher(dict(algorithm = 0, trees = 3), dict(checks = 1000)) 
        self.dest = [] 
        for i in range(self.tml): 
            self.tem.append(cv2.imread('/home/puzzlebot/catkin_ws/src/sign_detector/src/%d.jpeg' % i)) 
            self.tem[i] = np.pad(cv2.pyrDown(self.tem[i]), pad_width=[(50, 50),(50, 50),(0, 0)], mode='constant',constant_values=(255))
            _, destemp = self.orb.detectAndCompute(self.tem[i], None) 
            self.dest.append(np.float32(destemp)) 
        self.signals = ['stop', 'continue', 'round','turn', 'no speed limit','no'] 
        self.slml = [] 
        self.il = [] 
        self.ila = "" 
        self.slmp = 0 
        self.sizel = 15 
        self.continuar = False
        self.last = 5
        rospy.init_node('Signal_Detector') 
        rospy.Subscriber('/video_source/raw', Image, self.img_callback) 
        rospy.Subscriber('/activator', String, self.activator_callback) 
        self.signal_pub = rospy.Publisher('/img_processing/signal', String, queue_size=10)
        self.signal_idx_pub = rospy.Publisher('/img_processing/signal_idx', UInt8, queue_size=10) 
        self.rate = rospy.Rate(1/self.dt) 
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback) 
        rospy.on_shutdown(self.stop) 
 
    def timer_callback(self, time): 
        # if not self.activate:  
        #     return 
        if self.continuar:
            self.check_trff_lgt() 
         
    def check_trff_lgt(self): 
        _, desf = self.orb.detectAndCompute(self.image_raw, None) 
        desf = np.array(np.float32(desf)) 
        slm = 0 
        index = 0 
        matches = [] 
        matchesMask = [] 
        for j in range(self.tml): 
            try:
                matches.append(self.flann.knnMatch(self.dest[j], desf, k=2)) 
                matchesMask.append([[0,0] for k in range(len(matches[j]))]) 
                for k,(m,n) in enumerate(matches[j]): 
                    if m.distance < 0.7*n.distance: 
                        matchesMask[j][k]=[1,0] 
                matchesMask[j] = np.array(matchesMask[j]) 
                if slm < np.sum(matchesMask[j][:,0]): 
                    slm = np.sum(matchesMask[j][:,0]) 
                    index = j 
                if slm == 0: 
                    index = -1 
            except:
                pass
        self.slml.append(slm) 
        self.il.append(index) 
        if len(self.slml) > self.sizel: 
            self.slml.pop(0) 
            self.il.pop(0) 
        self.slmp = np.mean(np.array(self.slml)) 
        vals, counts = np.unique(self.il, return_counts=True) 
        self.ila = self.signals[int(vals[np.argwhere(counts == np.max(counts))][0])] 
        self.slma = round(np.median(np.array(self.slml))) 
        idx = 5
        # if self.slmp >= 2.0: 
            # Mandar signals[self.ila] 
        if self.ila == "stop" and self.slmp >= 3.0: 
            self.signal_pub.publish("stop, "+"slmp: "+str(self.slmp)) 
            idx = 0
        elif self.ila == "continue": 
            self.signal_pub.publish("continue, "+"slmp: "+str(self.slmp)) 
            idx = 1
        elif self.ila == "round": 
            self.signal_pub.publish("round, "+"slmp: "+str(self.slmp)) 
            idx = 2
        elif self.ila == "turn": 
            self.signal_pub.publish("turn, "+"slmp: "+str(self.slmp)) 
            idx = 3
        elif self.ila == "no speed limit": 
            self.signal_pub.publish("no speed limit, "+"slmp: "+str(self.slmp)) 
            idx = 4
        else: 
            self.signal_pub.publish("No hay matches, "+"slmp: "+str(self.slmp)) 
        msg_t = UInt8()
        if self.last == idx:
            msg_t.data = idx
        else:
            msg_t.data = 5
        self.signal_idx_pub.publish(msg_t)
        # else:         
        #     self.signal_pub.publish("No detecta nada, "+"slmp: "+str(self.slmp)) 
        self.last = idx
     
    def img_callback(self,msg): 
        self.image_raw = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.continuar = True
    def activator_callback(self,msg): 
            if msg.data == "TL_activate": 
                self.activate = True 
            elif msg.data == "TL_deactivate": 
                self.activate = False 
 
    def run(self): 
        rospy.spin() 
     
    def stop(self): 
        rospy.loginfo("Stopping signal detector.") 
 
if __name__ == '__main__':
    sign_detect = Signal_Detector()
    try:
        sign_detect.run()
    except:
        pass