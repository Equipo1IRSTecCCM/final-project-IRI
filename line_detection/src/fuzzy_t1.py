#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np
import skfuzzy as sk
from csv import reader

x = 0
y = 0
theta = 0
x2 = 0
y2 = 0
theta2= 0
X = []
Y = []
Za = []
Zl = []

def evaluarIdx(x,v):
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
def getValues(da,dl):
    idx_l = evaluarIdx(Y,da)
    idx_a = evaluarIdx(X,dl)
    val_a = Za[idx_l][idx_a]
    val_l = Zl[idx_l][idx_a]
    return val_a,val_l
def poseCallback1(pose_message):
    global x
    global y
    global theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

def poseCallback2(pose_message):
    global x2
    global y2
    global theta2

    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta
    
##################################################################################


def go_to_goal (xgoal, ygoal):
    msg = Twist()

    while(True):
#a partir de aqui sirve para convertir la separacion angular entre las dos tortugas en un entero dtheta entre -180 y 180 grados
        if theta > math.pi:
            pm_theta = (theta-(2*math.pi))*(180/math.pi)
        else:
            pm_theta = theta*(180/math.pi)

        #print("theta: ", round(theta,3), "\t pm_theta: ", round(pm_theta,3))
        gtheta = (math.atan2(y2-y, x2-x))*(180/math.pi)
        #print("gtheta: ", round(gtheta,3))
        dtheta = gtheta - pm_theta
        
        if dtheta>180:
            dtheta = int(dtheta - 360)
        if dtheta<-180:
            dtheta = int(dtheta + 360)
        else:
            dtheta = dtheta
        #print("dtheta: ", int(dtheta))
#hasta aqui
#todo lo demas del goal to goal hay que reconstruirlo tomando en cuenta que en un codigo aparte se genera toda la superficie de control
#y se puede importar como una matriz para solo tomar los valores de cierta coordenada
        dlineal = abs(np.sqrt((x-x2)**2+(y-y2)**2))
        w,v = getValues(dtheta,dlineal)
        print(dtheta,dlineal,w,v)
        msg.angular.z = w
        msg.linear.x = v
        velocity_publisher.publish(msg)
        if dlineal < 0.01:
            break

##################################################################################
#suscripciones
if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic1 = "/turtle1/pose"
        pose_subscriber1 = rospy.Subscriber(position_topic1, Pose, poseCallback1)

        position_topic2 = "/turtle2/pose"
        pose_subscriber2 = rospy.Subscriber(position_topic2, Pose, poseCallback2)

        #Entrada
        with open('/home/zuramaru/Documents/catkin_ws/src/fusible/ejercicios_logica_difusa/control/src/az.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Za = np.array([list(map(float, sublist)) for sublist in list_z])

        with open('/home/zuramaru/Documents/catkin_ws/src/fusible/ejercicios_logica_difusa/control/src/lz.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Zl = np.array([list(map(float, sublist)) for sublist in list_z])
        
        with open('/home/zuramaru/Documents/catkin_ws/src/fusible/ejercicios_logica_difusa/control/src/x.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        x_temp = [list(map(float, sublist)) for sublist in list_z]
        X = np.array(x_temp[0])

        with open('/home/zuramaru/Documents/catkin_ws/src/fusible/ejercicios_logica_difusa/control/src/y.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        y_temp = np.array([list(map(float, sublist)) for sublist in list_z])
        y_temp = np.transpose(y_temp)
        Y = np.array(y_temp[0])

        time.sleep(1.0)
        while(True):         
            
            go_to_goal(x2,y2)
                   
    except rospy.ROSInterruptException:        
        pass
