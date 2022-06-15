#!/usr/bin/env python
import numpy as np
import math
import rospy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
class odom_listener:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        #self.reinit_publisher = rospy.Publisher('/odom_reinit',UInt8, queue_size=10)
        self.reset_done = False
        rospy.Subscriber('/odom', Pose2D, self.odom_callback)
    def odom_callback(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta 

    def reset(self):
        msg = UInt8()
        msg.data=1
        if not self.reset_done:
            #self.reinit_publisher.publish(msg)
            pass
        self.reset_done = True
class image_listener:
    def __init__(self):
        #Twist cmd_vel
        self.w = 0
        self.v = 0
        self.w_e = 0
        self.v_e = 0
        self.lines = 0
        #Uint8 semaforo
        self.sem_idx = 4
        self.sem_select = 0
        #Uint8 Senal
        self.sig_idx = 5
        self.sig_last_diff = 5
        #Uint8 pasoZebra
        self.enPasoZebra = 0
        self.sem_toggle_publisher = rospy.Publisher('/img_processing/sem_toggle',UInt8, queue_size=10)
        rospy.Subscriber('/img_processing/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/img_processing/edge_cmd_vel', Twist, self.edge_cmd_vel_callback)
        rospy.Subscriber('/img_processing/zebra',UInt8, self.zebra_callback)
        rospy.Subscriber('/img_processing/sem_Data',UInt8, self.semaforo_callback)
        rospy.Subscriber('/img_processing/signal_idx',UInt8, self.signal_callback)
        rospy.Subscriber('/img_processing/lines',UInt8, self.lines_callback)
    def signal_callback(self,msg):
        
        self.sig_idx = msg.data
        if msg.data not in [5]:
            names = ["stop","cont","turn","round","nos"]
            if self.sem_select == 0 and msg.data in [1]:
                self.sig_last_diff = msg.data
                print(names[msg.data])
            elif self.sem_select == 1 and msg.data in [0,3,4]:
                self.sig_last_diff = msg.data
                print(names[msg.data])
            
            
    def lines_callback(self,msg):
        self.lines = msg.data
    def cmd_vel_callback(self,msg):
        self.w = msg.angular.z
        self.v = msg.linear.x
    def edge_cmd_vel_callback(self,msg):
        self.w_e = msg.angular.z
        self.v_e = msg.linear.x
    def zebra_callback(self,msg):
        self.enPasoZebra = bool(msg.data)
    def semaforo_callback(self,msg):
        self.sem_idx = msg.data
        #pass
class pp_listener:
    def __init__(self):
        #Twist cmd_vel
        self.w = 0
        self.v = 0
        self.pp_state = 0
        rospy.Subscriber('/pp/finish', UInt8, self.finish_callback)
        rospy.Subscriber('/pp/cmd_vel', Twist, self.cmd_vel_callback)
        self.pp_init_publisher = rospy.Publisher('/pp/points', Float32MultiArray, queue_size=10)
    def finish_callback(self,msg):
        self.pp_state = 2
    def cmd_vel_callback(self,msg):
        self.w = msg.angular.z
        self.v = msg.linear.x
    def invocar_pp(self,wp,type):
        msg_pp = Float32MultiArray()
        msg_pp.data = [wp[0],wp[1],type]
        print(msg_pp.data)
        self.pp_state = 1
        self.pp_init_publisher.publish(msg_pp)
        
class pilot:
    def __init__(self):
        rospy.init_node("pilot")
        self.odom = odom_listener()
        self.img_pross = image_listener()
        self.pp = pp_listener()
        self.dt = 0.1
        self.wp_set = False
        self.wait_point = [0,0]
        self.timer_max = 35
        self.timer = self.timer_max + 1
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)
        self.twist_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.fuzzy = False
        self.fuzzy_timer = 0
        self.estancado = False
        self.zebraWait = False
        self.zebraTimer = 0
        self.nos = 0
        self.iniciar = False
        rospy.on_shutdown(self.stop)
    def stop(self):
        print("Conductor down")
    def timer_callback(self,time):
        y_bias = 0.15
        msg = Twist()
        #Checar si paso de zebra
        if self.img_pross.enPasoZebra and self.pp.pp_state != 1:
            self.estancado = True
            print("zebra", "x ",self.odom.x," y ",self.odom.y)
            if self.zebraTimer == 0:
                self.zebraWait = True
            else:
                self.zebraWait = False
            self.zebraTimer += 1
        else:
            self.zebraWait = False
            self.zebraTimer = 0
        #print(self.img_pross.sem_idx)
        #Checar semaforo
        if not self.zebraWait:
            if self.estancado:
                if self.img_pross.sem_idx == 0:
                    self.estancado = True
                    print("alto")
                elif self.img_pross.sem_idx == 1:
                    self.estancado = True
                    print("alto")
                elif self.img_pross.sem_idx == 2:
                    #if self.pp.pp_state == 1:
                    if self.img_pross.sem_select==0:
                        if self.img_pross.sig_last_diff == 1:# or self.img_pross.sem_select==0:
                            if self.pp.pp_state != 1:
                                self.pp.invocar_pp([self.odom.x + 0.55,0.0],0.0)
                                if not self.wp_set:
                                    self.wait_point = [self.odom.x-0.30,self.odom.y+0.15]
                                    self.wp_set = True
                                    print("wp",self.wait_point)
                                msg_t = UInt8()
                                self.img_pross.sem_toggle_publisher.publish(msg_t)
                                self.img_pross.sem_select = 1
                                self.img_pross.sig_last_diff = 5
                            #self.estancado = False
                    #elif self.img_pross.sig_last_diff == 3 or self.img_pross.sem_select==1:
                    else:
                        if self.img_pross.sig_last_diff == 3:
                            if self.pp.pp_state != 1:
                                #self.odom.reset()
                                #self.pp.invocar_pp([self.odom.x-0.3,self.odom.y-0.2],2.0)
                                self.pp.invocar_pp(self.wait_point,2.0)
                                self.fuzzy = True
                                #self.pp.pp_state = 1
                                pass
                    print("siga",self.img_pross.sig_last_diff,self.pp.pp_state)
        if self.img_pross.sig_last_diff == 0:
            self.estancado = True
        
            
        if self.pp.pp_state == 2:
            print("pp end")
            self.estancado = False
            self.pp.pp_state = 0
        elif self.pp.pp_state == 1:
            self.estancado = False
            if self.img_pross.lines < 4 and self.fuzzy and self.fuzzy_timer < 5:
                self.pp.pp_state = 2
                self.fuzzy = False
                print("lines retake",self.img_pross.lines)
            if self.fuzzy:
                self.fuzzy_timer +=1
        #Definir que mensajede velocidad enviar
        if self.estancado:
            msg.linear.x = 0
            msg.angular.z = 0
            #Invocar pure pursuit
            print("Noni",self.pp.pp_state,self.odom.y < y_bias)
            
        else:
            #Enviar pure pursuit
            if self.pp.pp_state == 1:
                msg.linear.x = self.pp.v
                msg.angular.z = self.pp.w
            #Enviar edge
            elif self.timer < self.timer_max:
                msg.linear.x = self.img_pross.v_e
                msg.angular.z = self.img_pross.w_e
                self.timer_max += 1
            #Enviar camara
            else:
                msg.linear.x = self.img_pross.v
                msg.angular.z = self.img_pross.w
        if self.img_pross.sig_last_diff == 4 and self.nos < 10:
            msg.linear.x *= 3
            self.nos += 1
        if self.img_pross.sig_last_diff == 0:
            msg.linear.x = 0
            msg.angular.z = 0
        if self.img_pross.sig_last_diff == 1:
            self.iniciar = True
        if self.iniciar:
            self.twist_publisher.publish(msg)
        
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    toreto = pilot()
    try:
        toreto.run()
    except:
        pass
