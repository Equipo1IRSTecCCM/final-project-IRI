#!/usr/bin/env python

from re import S
import rospy
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import numpy as np

class purePursuit:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

        self.dt = 0.1
        
        rospy.init_node('pure_pursuit')

        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        #Definir el rate de rospy
        self.rate = rospy.Rate(10)
        #Indicar que hacer cuando se acabe
        rospy.on_shutdown(self.stop)

        self.initPos = [0, 0]
        self.distance_threshold = 0.1
        self.goal = [0,0]
        self.wp = []
        self.steps = 10

        self.wpCercano = 0
        self.wpDist = 0.0

        self.max_v = 0.3
        self.max_w = np.pi
        self.l_d = 0.5
        self.tw = 0.18
        self.r = 0.1 / 2
        self.velAuto = [0,0]
        self.wpLocal = []


    def timer_callback(self,time):
        self.pp()
        
        msg = Twist()
        msg.linear.x = self.velAuto[0]
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self.velAuto[1]
        #publicar
        self.twist_publisher.publish(msg)

    def crearLinea(self):
        tamano = 1
        self.goal[0] = tamano
        temp_x = np.transpose(np.array(np.linspace(self.initPos[0],self.goal[0],self.steps)))
        temp_y = np.transpose(np.array(np.linspace(self.initPos[1],self.goal[1],self.steps)))
        temp_x = temp_x.tolist()
        temp_y = temp_y.tolist()
        
        for i in range(len(temp_x)):
            self.wp.append([temp_x[i],temp_y[i]])

    def crearCuadrado(self):
        tamano = 1
        #steps = 10
        goal = [[self.x,self.y],[tamano,0],[tamano, tamano],[0,tamano],[0,0]]
        temp_x = np.zeros(self.steps*4-7)
        temp_y = np.zeros(self.steps*4-7)
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
            self.wp.append([temp_x[i],temp_y[i]])
    def crearTriangulo(self):
            tamano = 1
            angle = 60 * np.pi / 180
            goal = [[0,0],[tamano,0],[tamano/2, tamano * np.sin(angle)],[0,0]]
            
            temp_x = np.zeros(self.steps*3-5)
            temp_y = np.zeros(self.steps*3-5)
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
                self.wp.append([temp_x[i],temp_y[i]])
    def crearTrapecio(self):
        tamano = 1
        angle = 75 * np.pi / 180
        goal = [[0,0],[tamano,0],[tamano - tamano * np.cos(angle), tamano * np.sin(angle)],[tamano * np.cos(angle),tamano * np.sin(angle)],[0,0]]
        steps_temp = np.floor(self.steps/4)
        temp_x = np.zeros(self.steps*4-7)
        temp_y = np.zeros(self.steps*4-7)
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
            self.wp.append([temp_x[i],temp_y[i]])
    def obtenerDistancia(self):
        distancias = []
        for i in range(len(self.wp)):
            distancias.append(np.sqrt((self.x - self.wp[i][0])**2 + (self.y - self.wp[i][1])**2))
        return distancias
    def matlab(self):
        
        for i in range(len(self.wp)):
            print("{0},{1};".format(self.wp[i][0],self.wp[i][1]))
    def encontrarSiguiente(self):
        dist = np.asarray(self.obtenerDistancia()) #Calcular distancia
        #Sacar las que tengan un indice menor al que estamos
        #print(dist)
        maxDist = 0
        maxIndex = self.wpCercano
        #print(dist)
        sup = int(self.wpCercano + len(dist)/2)
        if(sup > len(dist)):
            sup = len(dist)
        for i in range(self.wpCercano,sup):       
            #Sacar los que se pasan de un l_d
            if(dist[i] <= self.l_d):
                #Sacar la de mayor distancia de lo que queda
                if (dist[i]>maxDist):
                    maxDist = dist[i]
                    maxIndex = i 
        self.wpCercano = maxIndex
        self.wpDist = dist[maxIndex]
        print("pos:",self.x, self.y)
        print("Sig:",self.wp[maxIndex],self.wpCercano,self.wpDist)
    def getOmega(self):
        #obtener alfa
        alpha = np.arctan2((self.wp[self.wpCercano][1]-self.y),(self.wp[self.wpCercano][0]-self.x)) - self.theta
        #Rotar
        R_mat = np.array([[np.cos(alpha),-np.sin(alpha)],[np.sin(alpha), np.cos(alpha)]])
        wpRot = np.matmul(R_mat,np.array([[self.wp[self.wpCercano][0]],[self.wp[self.wpCercano][1]]]))
        #Curvatura
        ik = 2 * (-wpRot[0][0]+wpRot[1][0]) / ((self.l_d)**2)
        #Sacar omega
        omega = ik * self.max_v
        if(abs(omega) > self.max_w):
            omega = self.max_w * abs(omega) / omega
        return omega 
        

    def pp(self):
        self.encontrarSiguiente() #Encontrar el siguiente punto
        if self.wpDist > 0.1:
            self.velAuto[0] = self.max_v
            self.velAuto[1] = self.getOmega()
        else:
            self.velAuto[0] = 0.0
            self.velAuto[1] = 0.0


    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta    

    def run(self):
        self.crearCuadrado()
        rospy.spin()
        while not rospy.is_shutdown():
            if self.wpDist < 0.1:
                self.stop()
            self.rate.sleep()
        

    def stop(self):
        rospy.loginfo("Pure pursuit ha cumplido con la patria")
        self.t1.shutdown()
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #publicar
        self.twist_publisher.publish(msg)
        print("Pure pur sad")


if __name__ == "__main__":
    pursuit = purePursuit()
    try:
        pursuit.run()
    except:
        pass