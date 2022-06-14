#!/usr/bin/env python

from re import S
import csv
import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8

import numpy as np

class purePursuit:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom_ready = True
        self.right = False
        self.dt = 0.01
        
        rospy.init_node('pure_pursuit')
        rospy.Subscriber('/odom', Pose2D, self.odom_callback) 
        rospy.Subscriber('/pp/points', Float32MultiArray, self.points_callback)
        rospy.Subscriber('/pp/init', UInt8, self.points_callback)
        rospy.Subscriber('/odom_ready', UInt8, self.ready_callback)
        self.twist_publisher = rospy.Publisher('/pp/cmd_vel', Twist, queue_size=10)
        self.finish_publisher = rospy.Publisher('/pp/finish', UInt8, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        #Definir el rate de rospy
        self.rate = rospy.Rate(10)
        #Indicar que hacer cuando se acabe
        rospy.on_shutdown(self.stop)

        self.initPos = [0, 0]
        self.distance_threshold = 0.1
        self.goal = [0,0]
        self.wpx = [10]
        self.wpy = [10]
        self.steps = 20

        self.wpCercano = 0
        self.wpDist = 0.0
        self.manejar = False
        #print(type(self.wp))
        #self.goal = np.array([self.init, [1, 0],[1,1],[0,1],[0,self.distance_threshold * 2]])

        self.max_v = 0.1
        self.max_w = 0.2
        self.l_d = 3
        #0.5
        self.r = 0.1 / 2
        self.velAuto = [0,0]
        self.wpLocal = []

    def ready_callback(self,msg):
        self.odom_ready = True
    def points_callback(self,msg):
        print("Me invocaron, incializando")
        temp = msg.data
        step_distance = 0.1
        if temp[2]==1.0:
            delta = [abs(temp[0]-self.x),abs(temp[1]-self.y)]
        else:
            delta = [abs(temp[0]-self.x),abs(temp[1]-self.y)]
        
        steps = [int(math.ceil(delta[0]/step_distance)),int(math.ceil(delta[1]/step_distance))]
        print("lol",steps,temp,[self.x,self.y])
        temp_x = []
        temp_y = []
        idx_arr = 0
        
        if steps[0]!= 0:
            if temp[2]==1.0:
                #linea_x = np.array(np.linspace(0.0,temp[0],steps[0]))
                #linea_y = np.array(np.linspace(0.0,0.0,steps[0]))
                linea_x = np.array(np.linspace(self.x,temp[0],steps[0]))
                linea_y = np.array(np.linspace(self.y,self.y,steps[0]))
            else:
                linea_x = np.array(np.linspace(self.x,temp[0],steps[0]))
                linea_y = np.array(np.linspace(self.y,self.y,steps[0]))
            if len(linea_x) > 2:
                linea_x = np.delete(linea_x,0,0)
                linea_y = np.delete(linea_y,0,0)
                linea_x = np.delete(linea_x,len(linea_x)-1,0)
                linea_y = np.delete(linea_y,len(linea_y)-1,0)
            for idx in range(len(linea_x)):
                temp_x.append(linea_x[idx])
                temp_y.append(linea_y[idx])
                idx_arr += 1
        if steps[1]!= 0:
            if temp[2]==1.0:
                #linea_x = np.array(np.linspace(temp[0],temp[0],steps[1]))
                #linea_y = np.array(np.linspace(0.0,temp[1],steps[1]))
                linea_x = np.array(np.linspace(temp[0],temp[0],steps[1]))
                linea_y = np.array(np.linspace(self.y,temp[1],steps[1]))
            else:    
                linea_x = np.array(np.linspace(self.x,self.x,steps[1]))
                linea_y = np.array(np.linspace(self.y,temp[1],steps[1]))
            if len(linea_x) > 2:
                linea_x = np.delete(linea_x,0,0)
                linea_y = np.delete(linea_y,0,0)
                linea_x = np.delete(linea_x,len(linea_x)-1,0)
                linea_y = np.delete(linea_y,len(linea_y)-1,0)
            
            if temp[2] == 1.0:
                for idx in range(len(linea_x)):
                    temp_x.append(linea_x[idx])
                    temp_y.append(linea_y[idx])
                    idx_arr += 1
        temp_x = np.array(temp_x)
        temp_y = np.array(temp_y)
        self.wpx = []
        self.wpy = []
        temp_x = np.transpose(temp_x).tolist()
        temp_y = np.transpose(temp_y).tolist()
        for i in range(len(temp_x)):
            self.wpx.append(temp_x[i])
            self.wpy.append(temp_y[i])
        if temp[2] == 1.0:
            self.right = True
            self.l_d = 0.7
            #self.wpx = [0.4]
            #self.wpy = [0.0]
        else:
            self.l_d = 3
        print("pp invocado",temp)
        if temp[2] != 2.0:
            self.manejar = not self.manejar
        print(self.wpx,self.wpy)
    def timer_callback(self,time):
        if self.manejar:
            idx = self.nextPunto()
            
            xp = self.wpx[idx]
            yp = self.wpy[idx]
            alpha = np.arctan2((yp-self.y),(xp-self.x))-self.theta

        
            xRot = xp * np.cos(alpha) - yp * np.sin(alpha)
            yRot = xp * np.sin(alpha) + yp* np.cos(alpha)
            xp = -xRot+self.x
            yp = yRot-self.y
            ik = 2 * (yp+xp) / (self.l_d**2)
            omega = ik * self.max_v
            if abs(omega) > self.max_w:
                omega = self.max_w * np.sign(omega)
            msg = Twist()
            msg.linear.x = self.max_v
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = omega
            
            #publicar
            t = self.right and self.odom_ready
            t = t or not self.right
            if t:
                print("pos",self.x,self.y)
                print("next",self.wpx[idx],self.wpy[idx])
                self.twist_publisher.publish(msg)
    def nextPunto(self):
        dx = [self.x - ipx for ipx in self.wpx]
        dy = [self.y - ipy for ipy in self.wpy]
        d = np.hypot(dx,dy)
        idx = np.argmin(d) + 1 if np.argmin(d) + 1 < len(self.wpx) else len(self.wpx) - 1
        return idx

    def obtenerDistancia(self):
        distancias = []
        for i in range(len(self.wp)):
            distancias.append(np.sqrt((self.x - self.wp[i][0])**2 + (self.y - self.wp[i][1])**2))
        return distancias
    def matlab(self):
        
        for i in range(len(self.wpx)):
            print("{0},{1};".format(self.wpx[i],self.wpy[i]))

    def getPoints(self):
        #0.0,0.0
        #2.08,0
        #2.08,1.20
        #0.88,1.20
        #0.88,0
        #0.0,0.0
        goal = [[0.1,0.0],[1.8,0.0],[1.8,1],[0.8,1],[0.1,0.1]]
        print(goal)
        lados = len(goal)-1
        temp_x = np.zeros(self.steps*lados-2*lados)
        temp_y = np.zeros(self.steps*lados-2*lados)
        for i in range(len(goal)-1):
            
            linea_x = np.array(np.linspace(goal[i][0],goal[i+1][0],self.steps))
            linea_y = np.array(np.linspace(goal[i][1],goal[i+1][1],self.steps))
            linea_x = np.delete(linea_x,0,0)
            linea_y = np.delete(linea_y,0,0)
            linea_x = np.delete(linea_x,len(linea_x)-1,0)
            linea_y = np.delete(linea_y,len(linea_y)-1,0)
            for idx in range(len(linea_x)):
                temp_x[i*self.steps + idx - 2 * i] = linea_x[idx]
                temp_y[i*self.steps + idx - 2 * i] = linea_y[idx]
        
        temp_x = np.transpose(temp_x).tolist()
        temp_y = np.transpose(temp_y).tolist()
        print(len(temp_x))
        for i in range(len(temp_x)):
            self.wpx.append(temp_x[i])
            self.wpy.append(temp_y[i])
        print(self.wpx)
        

    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta    

    def run(self):
        #self.crearCuadrado()
        #self.crearTrapecio()
        #self.readPath(1)
        #self.temp()
        #self.matlab()
        #self.getPoints()
        while not rospy.is_shutdown():
            dx = self.x - self.wpx[-1]
            dy = self.y - self.wpy[-1]
            d = np.hypot(dx,dy)
            #print("dist",d)
            if d < 0.1:
                #self.stop()
                self.manejar = False
                msg = UInt8()
                msg.data = 1
                self.wpx[-1]=100
                self.finish_publisher.publish(msg)
            #self.rate.sleep()
        

    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #publicar
        self.t1.shutdown()
        self.twist_publisher.publish(msg)
        print("Pure pur sad")


if __name__ == "__main__":
    pursuit = purePursuit()
    try:
        pursuit.run()
    except:
        pass
