#!/usr/bin/env python

from cmath import cos
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class odometry:
    def __init__(self):
        self.wr = 0.0
        self.wl = 0.0
        #derivadas
        self.xdot = 0.0
        self.thetadot = 0.0

        self.x = 0.0
        self.theta = 0
        #dimensiones robot
        self.r = 0.05
        self.l = 0.18
        #Variables goblales
        self.global_X = 0.0
        self.global_Y = 0.0
        self.theta = 0.0

        #timer
        self.dt = 0.1 #s

        #init rospy
        rospy.init_node('odometry')

        rospy.Subscriber('/wr', Float32, self.wr_callback)
        rospy.Subscriber('/wl', Float32, self.wl_callback)
        self.odom_publisher = rospy.Publisher('/odom', Pose2D, queue_size=10)
        #Crear timer
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        #Definir el rate de rospy
        self.rate = rospy.Rate(10)
        #Indicar que hacer cuando se acabe
        rospy.on_shutdown(self.stop)

    def timer_callback(self,time):
        self.xdot = self.r * (self.wr + self.wl) / 2.0
        self.thetadot = self.r * (self.wr - self.wl) / self.l
        self.x += self.xdot * self.dt
        #integrar thetadot con integral de euler
        self.theta += self.thetadot * self.dt
        #integrar xdot con integral de euler y descomponer en ejes
        self.global_X += self.xdot * np.cos(self.theta) * self.dt
        self.global_Y += self.xdot * np.sin(self.theta) * self.dt
        msg = Pose2D()
        #publicar
        msg.x = self.global_X
        msg.y = self.global_Y
        msg.theta = self.theta
        self.odom_publisher.publish(msg)
        
    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data
    
    def run(self):
        rospy.spin()

    def stop(self):
        print("Odomo sad")

if __name__ == "__main__":
    odom = odometry()
    try:
        odom.run()
    except:
        pass
