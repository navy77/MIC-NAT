#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
import time
from rosserial_arduino.srv._Test import *
import numpy as np

class Mic_agv:
    def __init__(self):
        rospy.init_node("agv_mecanum",anonymous=True)
        self.rate = 10
        self.pwm_value = rospy.Publisher('/pwm_data',Float32MultiArray,queue_size=10)
        self.cmd_value = rospy.Subscriber('/cmd_vel',Twist,self.cb_cmd_vel)
        self.enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,self.cb_enc_value)
        self.ir_value = rospy.Subscriber('/ir_data',Int32MultiArray,self.cb_ir_value)
        self.imu_value = rospy.Subscriber('/imu_data',Float32,self.cb_imu_value)
        # cmd_vel
        self.vel_x,self.vel_y,self.vel_z = 0,0,0
        # mecanum wheel
        self.lx = 0.3
        self.ly = 0.3
        self.wheel_diameter = 0.127
        self.encoder_tick_0,self.encoder_tick_1 = 2835,3150
        # encoder data
        self.enc_list =[]
        self.prv_enc_list =[0,0,0,0]
        # self.enc_0,self.enc_1,self.enc_2,self.enc_3 = 0,0,0,0
        # self.prv_enc_0,self.prv_enc_1,self.prv_enc_2,self.prv_enc_3 = 0,0,0,0
        # pwm data
        self.pwm_list = []
        # self.pwm_0,self.pwm_1,self.pwm_2,self.pwm_3 = 0,0,0,0
        # infrared data
        self.ir_list = []
        # self.ir_0,self.ir_1 = 0,0
        # vel
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.vel_0,self.vel_1,self.vel_2,self.vel_3 = 0,0,0,0
        self.th = 0 
        self.imu = 0 # rad/s
        self.dist_list = []
        self.vel_list =[]
        self.vel_t_list =[]
        # PID
        self.kp = 10
        self.ki = 1
        self.kd = 1
        self.prv_err_list = [0,0,0,0]
        self.integral_list = [0,0,0,0]
        self.max_pwm = 250
        self.min_pwm = -250

    def cb_imu_value(self,msg):
        self.imu = msg.data

    def cb_cmd_vel(self,Twist):
        self.vel_x = Twist.linear.x
        self.vel_y = Twist.linear.y
        self.vel_z = Twist.angular.z
    
    def cb_ir_value(self,msg):
        self.ir_list = msg.data
        # self.ir_0 = msg.data[0]
        # self.ir_1 = msg.data[1]

    def cmd_cal(self):
        if self.vel_x == 0 and self.vel_y == 0 and self.vel_z == 0:
            self.vel_list = [0,0,0,0]
            self.vel_t_list = [0,0,0,0]
            self.pwm_list = [0,0,0,0]
        else:    
            vel_t_0 = self.vel_x-self.vel_y-(self.lx+self.ly)*self.vel_z
            vel_t_1 = self.vel_x+self.vel_y+(self.lx+self.ly)*self.vel_z 
            vel_t_2 = self.vel_x+self.vel_y-(self.lx+self.ly)*self.vel_z
            vel_t_3 = self.vel_x-self.vel_y+(self.lx+self.ly)*self.vel_z
            self.vel_t_list = [vel_t_0,vel_t_1,vel_t_2,vel_t_3]
        # print(self.vel_t_list)

    def low_pass_filter(self):
        pass

    def vel_calculate(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            dis_front = (np.array(self.enc_list[0:2])-np.array(self.prv_enc_list[0:2])) / self.encoder_tick_0
            dis_rear = (np.array(self.enc_list[2:4])-np.array(self.prv_enc_list[2:4])) / self.encoder_tick_1
            self.dist_list = np.concatenate((dis_front,dis_rear))

            self.vel_list = np.round(self.dist_list/elapsed,2)
            self.prv_enc_list = self.enc_list

            rospy.loginfo(self.vel_list)
            # print(self.vel_list)
            # print(self.prv_enc_list)
        
    def pid_calculate(self):
        self.vel_calculate()
        pwm = []
        err_list = np.array(self.vel_t_list) - np.array(self.vel_list)
        # Proportional term
        p = self.kp * err_list
        # # Integral term
        self.integral_list = np.array(self.integral_list) + np.array(err_list)
        i = self.ki * self.integral_list
        # Derivative term
        derivative = err_list - self.prv_err_list
        d = self.kd * derivative
        pwm = p + i + d

        for i in range(4):
            if self.vel_t_list[i] == 0:
                pwm[i] = 0

        for i in range(4):
            if pwm[i]>self.max_pwm:
                pwm[i] = self.max_pwm

        for i in range(4):
            if pwm[i]<self.min_pwm:
                pwm[i] = self.min_pwm   
        
        self.pwm_list = Float32MultiArray()
        self.pwm_list.data = pwm

        self.prv_err_list = err_list

        self.pwm_value.publish(self.pwm_list)
        # rospy.loginfo(self.pwm_list)
      
        # print(self.vel_t_list)
        # print(self.vel_list)
        
    def cb_enc_value(self,msg):
        self.enc_list = msg.data
        # self.enc_0 = msg.data[0]
        # self.enc_1 = msg.data[1]
        # self.enc_2 = msg.data[2]
        # self.enc_3 = msg.data[3]

    def update(self):
        self.cmd_cal()
        self.pid_calculate()
        # print(self.vel_list)
        # print(self.vel_t_list)

    def spin(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # rospy.loginfo("%s"+":"+"%s"+":"+"%s"+":"+"%s",self.enc_0,self.enc_1,self.enc_2,self.enc_3)
            self.update()
            r.sleep()

if __name__ == '__main__':
    try:
        mic = Mic_agv()
        mic.spin()
    except rospy.ROSInterruptException:
        pass
