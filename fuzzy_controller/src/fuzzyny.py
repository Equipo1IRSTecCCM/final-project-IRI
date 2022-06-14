#! /usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray
import math
from csv import reader
class fuzzy:
    def __init__(self):
        
        self.x = 0
        self.y = 0
        self.theta = 0

        self.X = []
        self.Y = []
        self.Za = []
        self.Zl = []

        self.xgoal = 0
        self.ygoal = 0

        self.manejar = False

        self.dt = 0.01
        rospy.init_node("fuzzy_cnt")
        rospy.Subscriber('/pp/points', Float32MultiArray, self.points_callback)
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.finish_publisher = rospy.Publisher('/pp/finish', UInt8, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

        self.twist_publisher = rospy.Publisher('/pp/cmd_vel', Twist, queue_size=10)
    def points_callback(self,msg):
        temp = msg.data
        if temp[2]==2.0:
            print("Me invocaron",temp)
            self.xgoal = temp[0]
            self.ygoal = temp[1]
            self.manejar = True
    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta 
    def evaluarIdx(self,x,v):
        """
        Find the index of v on x (or the closest value)
        
        Parameters
        ----------
        x :     numpy.array(float[])
            The array
        v :     float
            The searched value
        """
        idx = np.where(x==v)
        #In case it's empty
        if idx[0].shape == (0,):
            closest = 0 
            for i in x:
                if abs(v-i) < abs(v-closest):
                    closest = i
            idx = np.where(x==closest)
        #print(x)
        return idx[0][0]
    def getValues(self,p,d):
        idx_l = self.evaluarIdx(self.Y,p)
        idx_a = self.evaluarIdx(self.X,d)
        val_a = self.Za[idx_l][idx_a]
        val_l = self.Zl[idx_l][idx_a]
        return val_a,val_l
    def read(self):
        p = "/home/puzzlebot/catkin_ws/src/line_detection/src/"
        with open(p + 'az.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Za = np.array([list(map(float, sublist)) for sublist in list_z])

        with open(p + 'lz.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Zl = np.array([list(map(float, sublist)) for sublist in list_z])
        
        with open(p + 'x.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        x_temp = [list(map(float, sublist)) for sublist in list_z]
        X = np.array(x_temp[0])

        with open(p +'y.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        y_temp = np.array([list(map(float, sublist)) for sublist in list_z])
        y_temp = np.transpose(y_temp)
        Y = np.array(y_temp[0])
        return X, Y, Za, Zl

    def timer_callback (self,time):
        msg = Twist()
        dlineal = abs(np.sqrt((self.x-self.xgoal)**2+(self.y-self.ygoal)**2))
        if dlineal > 0.1 and self.manejar:
            if self.theta > math.pi:
                pm_theta = (self.theta-(2*math.pi))*(180/math.pi)
            else:
                pm_theta = self.theta*(180/math.pi)

            #print("theta: ", round(theta,3), "\t pm_theta: ", round(pm_theta,3))
            gtheta = (math.atan2(self.ygoal-self.y, self.xgoal-self.x))*(180/math.pi)
            #print("gtheta: ", round(gtheta,3))
            dtheta = gtheta - pm_theta
            
            if dtheta>180:
                dtheta = int(dtheta - 360)
            if dtheta<-180:
                dtheta = int(dtheta + 360)
            else:
                dtheta = dtheta
            #print("dtheta: ", int(dtheta))

            print("pos",self.x,self.y)
            #w,v = getValues(dtheta,dlineal)
            dtheta *= np.pi/180
            w = dtheta * 0.07
            v = dlineal * 0.1
            if dlineal < 0.1:
                self.manejar = False
                msg_t = UInt8()
                self.finish_publisher.publish(msg_t)
            print(dtheta,dlineal,w,v)
            msg.angular.z = w
            msg.linear.x = v
            self.twist_publisher.publish(msg)
            
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
    fuzzynator = fuzzy()
    try:
        fuzzynator.run()
    except:
        pass