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
        self.theta = -np.pi/2

        self.dt = 0.01
        
        rospy.init_node('pure_pursuit')

        rospy.Subscriber('/pp/points', Float32MultiArray, self.points_callback)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_publisher = rospy.Publisher('/pp/finish', UInt8, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        #Definir el rate de rospy
        self.rate = rospy.Rate(10)
        #Indicar que hacer cuando se acabe
        rospy.on_shutdown(self.stop)

        self.initPos = [0, 0]
        self.distance_threshold = 0.1
        self.goal = [0,0]
        self.wpx = []
        self.wpy = []
        self.steps = 10

        self.wpCercano = 0
        self.wpDist = 0.0
        self.manejar = False
        #print(type(self.wp))
        #self.goal = np.array([self.init, [1, 0],[1,1],[0,1],[0,self.distance_threshold * 2]])

        self.max_v = 0.1
        self.max_w = np.pi/2
        self.l_d = 1
        #0.5
        self.r = 0.1 / 2
        self.velAuto = [0,0]
        self.wpLocal = []
    def points_callback(self,msg):
        temp = msg.data
        for px,py in temp:
            self.wpx.append(px)
            self.wpy.append(py)
        self.manejar = True

    def timer_callback(self,time):
        if self.manejar:
            idx = self.nextPunto()
            print(idx)
            xp = self.wpx[idx]
            yp = self.wpy[idx]
            alpha = np.arctan2((yp-self.y),(xp-self.x))-self.theta

        
            xRot = xp * np.cos(alpha) - yp * np.sin(alpha)
            yRot = xp * np.sin(alpha) + yp* np.cos(alpha)
            xp = -xRot+self.x
            yp = yRot-self.y
            ik = 2 * (yp+xp) / (self.l_d**2)
            omega = ik * self.max_v
            
            msg = Twist()
            msg.linear.x = self.max_v
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = omega
            #publicar
            self.twist_publisher.publish(msg)
    def nextPunto(self):
        dx = [self.x - ipx for ipx in self.wpx]
        dy = [self.y - ipy for ipy in self.wpy]
        d = np.hypot(dx,dy)
        idx = np.argmin(d) + 1 if np.argmin(d) + 1 < len(self.wpx) else len(self.wpx) - 1
        return idx
    def crearLinea(self):
        tamano = 1
        self.goal[0] = tamano
        temp_x = np.transpose(np.array(np.linspace(self.initPos[0],self.goal[0],self.steps)))
        temp_y = np.transpose(np.array(np.linspace(self.initPos[1],self.goal[1],self.steps)))
        temp_x = temp_x.tolist()
        temp_y = temp_y.tolist()
        
        for i in range(len(temp_x)):
            self.wpx.append(temp_x[i])
            self.wpy.append(temp_y[i])

    def crearCuadrado(self):
        tamano = 1
        #steps = 10
        goal = [[0.1,0],[tamano,0],[tamano, tamano],[0,tamano],[0,0.1]]
        temp_x = np.zeros(self.steps*4-8)
        temp_y = np.zeros(self.steps*4-8)
        for i in range(4):
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
        #print(self.wp[:][1])
    def crearTriangulo(self):
        tamano = 1
        angle = 60 * np.pi / 180
        goal = [[0,0],[tamano,0],[0.5, tamano * np.sin(angle)],[0.1 * np.cos(angle),0.1 * np.sin(angle)]]
        print(goal)
        temp_x = np.zeros(self.steps*3-6)
        temp_y = np.zeros(self.steps*3-6)
        for i in range(3):
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
        for i in range(len(temp_x)):
            self.wpx.append(temp_x[i])
            self.wpy.append(temp_y[i])
    def crearTrapecio(self):
        tamano = 1
        angle = 75 * np.pi / 180
        goal = [[0,0],[tamano,0],[tamano - tamano * np.cos(angle), tamano * np.sin(angle)],[tamano * np.cos(angle),tamano * np.sin(angle)],[0,0]]
        steps_temp = np.floor(self.steps/4)
        temp_x = np.zeros(self.steps*4-8)
        temp_y = np.zeros(self.steps*4-8)
        for i in range(4):
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
        for i in range(len(temp_x)):
            self.wpx.append(temp_x[i])
            self.wpy.append(temp_y[i])
    def readPath(self,numTra):
        
        names = ["/home/puzzlebot/catkin_ws/src/pur_pur/src/aStar.csv", "/home/puzzlebot/catkin_ws/src/pur_pur/src/prm.csv","/home/puzzlebot/catkin_ws/src/pur_pur/src/rrt.csv"]
        print('func init')
        with open(names[numTra],'r') as f:
            print('opened')
            rows = list(csv.reader(f))
        #print(rows)
        for row in rows:
            print(row[0])
            self.wpx.append(float(row[0]))
            self.wpy.append(float(row[1]))
        print(rows)
        
    def temp(self):
        #RRT
        #wp = [[0.1, 2.4], [0.128942, 2.308477], [0.133095, 2.233091], [0.154117, 2.135331], [0.158984, 2.064384], [0.155652, 1.969236], [0.204411, 1.881966], [0.279901, 1.825973], [0.326941, 1.737733], [0.312394, 1.678586], [0.372437, 1.63909], [0.452458, 1.616166], [0.4604, 1.517844], [0.535371, 1.452129], [0.62576, 1.409982], [0.622815, 1.334351], [0.68013, 1.332661], [0.644045, 1.254882], [0.694652, 1.211421], [0.761128, 1.187024], [0.735825, 1.111195], [0.720847, 1.013528], [0.766441, 0.926601], [0.861537, 0.900528], [0.886294, 0.834558], [0.929197, 0.747379], [0.967595, 0.655586], [0.928917, 0.564265], [0.979144, 0.478245]]
        #PRM
        #wp = [[0.1, 2.4],[0.210234, 2.363022],[0.316595, 2.315914],[0.421873, 2.266456],[0.521559, 2.206609],[0.57808, 2.165752],[0.494619, 2.247272],[0.42122, 2.337957],[0.402955, 2.360876],[0.453382, 2.25567],[0.489327, 2.152025],[0.464608, 2.266043],[0.461591, 2.261938],[0.459274, 2.145373],[0.443405, 2.029869],[0.414201, 1.916998],[0.372059, 1.808293],[0.318908, 1.70448],[0.272668, 1.59741],[0.239038, 1.484962],[0.218814, 1.369348],[0.212276, 1.25216],[0.219514, 1.135014],[0.232968, 1.051426],[0.198667, 1.163672],[0.15145, 1.271125],[0.150284, 1.278661],[0.205853, 1.176145],[0.24911, 1.067857],[0.279467, 0.95527],[0.300389, 0.840505],[0.331714, 0.728183],[0.375902, 0.620272],[0.432351, 0.518238],[0.467172, 0.473681],[0.545983, 0.387217],[0.634354, 0.310551],[0.731073, 0.244729],[0.834818, 0.190655],[0.944169, 0.149066],[1.057629, 0.120534],[1.173644, 0.105448],[1.084095, 0.105126],[0.967745, 0.092886],[1.060884, 0.115057],[1.094355, 0.125334],[0.980914, 0.095552],[1.010571, 0.096457],[1.127828, 0.099041],[1.244581, 0.087878],[1.2, 0.1]]
        #aStar
        wp = [[ 0.05 , 2.4000000000000004 ],[ 0.1 , 2.35 ],[ 0.15000000000000002 , 2.3000000000000003 ],[ 0.15000000000000002 , 2.25 ],[ 0.2 , 2.2 ],[ 0.25 , 2.1500000000000004 ],[ 0.30000000000000004 , 2.1 ],[ 0.30000000000000004 , 2.0500000000000003 ],[ 0.30000000000000004 , 2.0 ],[ 0.35000000000000003 , 1.9500000000000002 ],[ 0.4 , 1.9000000000000001 ],[ 0.45 , 1.85 ],[ 0.5 , 1.8000000000000003 ],[ 0.55 , 1.75 ],[ 0.6000000000000001 , 1.7000000000000002 ],[ 0.65 , 1.6500000000000001 ],[ 0.65 , 1.6 ],[ 0.65 , 1.5500000000000003 ],[ 0.65 , 1.5 ],[ 0.65 , 1.4500000000000002 ],[ 0.65 , 1.4000000000000001 ],[ 0.65 , 1.35 ],[ 0.65 , 1.3 ],[ 0.7000000000000001 , 1.25 ],[ 0.7000000000000001 , 1.2000000000000002 ],[ 0.75 , 1.1500000000000001 ],[ 0.75 , 1.1 ],[ 0.8 , 1.05 ],[ 0.8500000000000001 , 1.0 ],[ 0.9 , 0.9500000000000002 ],[ 0.9500000000000001 , 0.9000000000000001 ],[ 1.0 , 0.8500000000000001 ],[ 1.05 , 0.8 ],[ 1.05 , 0.75 ],[ 1.05 , 0.7000000000000002 ],[ 1.05 , 0.6500000000000001 ],[ 1.05 , 0.6000000000000001 ],[ 1.05 , 0.55 ],[ 1.05 , 0.5 ],[ 1.05 , 0.4500000000000002 ],[ 1.05 , 0.3999999999999999 ],[ 1.1 , 0.3500000000000001 ],[ 1.1 , 0.30000000000000027 ],[ 1.1500000000000001 , 0.25 ],[ 1.1500000000000001 , 0.20000000000000018 ],[ 1.2000000000000002 , 0.1499999999999999 ],[ 1.2000000000000002 , 0.10000000000000009 ],[ 1.2000000000000002 , 0.04999999999999982 ],[ 1.25 , 0.0 ]]
        for i in wp:
            self.wpx.append(i[0])
            self.wpy.append(i[1])
    def obtenerDistancia(self):
        distancias = []
        for i in range(len(self.wp)):
            distancias.append(np.sqrt((self.x - self.wp[i][0])**2 + (self.y - self.wp[i][1])**2))
        return distancias
    def matlab(self):
        
        for i in range(len(self.wpx)):
            print("{0},{1};".format(self.wpx[i],self.wpy[i]))


   


    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta    

    def run(self):
        #self.crearCuadrado()
        #self.crearTrapecio()
        self.readPath(1)
        #self.temp()
        self.matlab()
        while not rospy.is_shutdown():
            dx = self.x - self.wpx[-1]
            dy = self.y - self.wpy[-1]
            d = np.hypot(dx,dy)
            if d < 0.1:
                self.stop()
            self.rate.sleep()
        

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