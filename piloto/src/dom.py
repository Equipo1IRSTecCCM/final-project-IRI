#!/usr/bin/env python

import numpy as np

import rospy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
class odom_listener:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
    def odom_callback(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta 
class image_listener:
    def __init__(self):
        #Twist cmd_vel
        self.w = 0
        self.v = 0
        #Uint8 semaforo
        self.sem_idx = 4
        #Uint8 pasoZebra
        self.enPasoZebra = 0
        rospy.Subscriber('/img_processing/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/img_processing/zebra',UInt8, self.zebra_callback)
        #rospy.Subscriber('/img_processing/sem_Data',UInt8, self.semaforo_callback)
    def cmd_vel_callback(self,msg):
        self.w = msg.angular.z
        self.v = msg.linear.x
    def zebra_callback(self,msg):
        self.enPasoZebra = bool(msg.data)
    def semaforo_callback(self,msg):
        self.sem_idx = msg.data
class pilot:
    def __init__(self):
        rospy.init_node("pilot")
        self.odom = odom_listener()
        self.img_pross = image_listener()
        self.dt = 0.1
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.estancado = False
        rospy.on_shutdown(self.stop)
    def stop(self):
        print("Conductor down")
    def timer_callback(self,time):
        msg = Twist()
        if self.img_pross.enPasoZebra:
            self.estancado = True
        if self.estancado:
            if self.img_pross.sem_idx == 0:
                self.estancado = True
            elif self.img_pross.sem_idx == 1:
                self.estancado = False
            elif self.img_pross.sem_idx == 2:
                self.estancado = False
        if self.estancado:
            msg.linear.x = 0
            msg.angular.z = 0
        else:
            msg.linear.x = self.img_pross.v
            msg.angular.z = self.img_pross.w
        self.twist_publisher.publish(msg)
        
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    toreto = pilot()
    try:
        toreto.run()
    except:
        pass
