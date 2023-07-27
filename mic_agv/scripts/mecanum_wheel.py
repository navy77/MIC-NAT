#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32,Float32,Float32MultiArray,Int32MultiArray
from geometry_msgs.msg import Twist
import time
from rosserial_arduino.srv._Test import *

class Mic_agv:
    def __init__(self):
        rospy.init_node("mic_node1",anonymous=True)
        self.rate = 20
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
        self.vel_t_0,self.vel_t_1,self.vel_t_2,self.vel_t_3 = 0.05,0,0,0
        # encoder data
        self.enc_0,self.enc_1,self.enc_2,self.enc_3 = 0,0,0,0
        self.prv_enc_0,self.prv_enc_1,self.prv_enc_2,self.prv_enc_3 = 0,0,0,0
        # pwm data
        self.pwm = []
        self.pwm_0,self.pwm_1,self.pwm_2,self.pwm_3 = 0,0,0,0
        # infrared data
        self.ir_0,self.ir_1 = 0,0
        # vel
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.vel_0,self.vel_1,self.vel_2,self.vel_3 = 0,0,0,0
        self.th = 0 
        self.imu = 0 # rad/s

    def cb_imu_value(self,msg):
        self.imu = msg.data

    def cb_cmd_vel(self,Twist):
        self.vel_x = Twist.linear.x
        self.vel_y = Twist.linear.y
        self.vel_z = Twist.angular.z

    def cmd_cal(self):
        if self.vel_x == 0 and self.vel_y == 0 and self.vel_z == 0:
            self.vel_0,self.vel_1,self.vel_2,self.vel_3 = 0,0,0,0
            self.pwm_0,self.pwm_1,self.pwm_2,self.pwm_3 = 0,0,0,0 
        else:
            self.vel_t_0 = self.vel_x-self.vel_y-(self.lx+self.ly)*self.vel_z
            self.vel_t_1 = self.vel_x+self.vel_y+(self.lx+self.ly)*self.vel_z 
            self.vel_t_2 = self.vel_x+self.vel_y-(self.lx+self.ly)*self.vel_z
            self.vel_t_3 = self.vel_x-self.vel_y+(self.lx+self.ly)*self.vel_z

    def vel_cal(self):
        acts = []
        flts = []
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.enc_0 == 0 and self.enc_1 == 0 and self.enc_2 == 0 and self.enc_3 == 0:
                distance_0 = 0
                distance_1 = 0
                distance_2 = 0
                distance_3 = 0
            else:
                distance_0 = (self.enc_0 - self.prv_enc_0)/self.encoder_tick_0
                distance_1 = (self.enc_1 - self.prv_enc_1)/self.encoder_tick_0
                distance_2 = (self.enc_2 - self.prv_enc_2)/self.encoder_tick_1
                distance_3 = (self.enc_3 - self.prv_enc_3)/self.encoder_tick_1
                self.vel_0 = distance_0/elapsed
                self.vel_1 = distance_1/elapsed
                self.vel_2 = distance_2/elapsed
                self.vel_3 = distance_3/elapsed
   
            self.prv_enc_0 = self.enc_0
            self.prv_enc_1 = self.enc_1
            self.prv_enc_2 = self.enc_2
            self.prv_enc_3 = self.enc_3
            # print(self.vel_0,distance_0,elapsed) 

    def pid_cal(self,kp,ki,kd):
        i_err_0,prv_err_0,d_err_0 = 0,0,0

        err_0 = self.vel_t_0-self.vel_0
        i_err_0 = i_err_0 + err_0
        d_err_0 = err_0 - prv_err_0

        self.pwm_0 = kp*err_0+ki*i_err_0+kd*d_err_0
        prv_err_0 = err_0
        
        
    def cb_enc_value(self,msg):
        self.enc_0 = msg.data[0]
        self.enc_1 = msg.data[1]
        self.enc_2 = msg.data[2]
        self.enc_3 = msg.data[3]

    def cb_ir_value(self,msg):
        self.ir_0 = msg.data[0]
        self.ir_1 = msg.data[1]

    def update(self):
        self.cmd_cal()
        self.vel_cal()
        self.pid_cal(500,100,100)
        self.pwm = [self.pwm_0,self.pwm_1,self.pwm_2,self.pwm_3]
        self.pwm_value.publish(Float32MultiArray(data = self.pwm))
        print(self.pwm_0,self.vel_0)

    def spin(self):
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # rospy.loginfo("%s"+":"+"%s"+":"+"%s"+":"+"%s",self.enc_0,self.enc_1,self.enc_2,self.enc_3)
            self.update()
            r.sleep()
if __name__ == '__main__':
    try:
        #rospy.init_node("mic_node1",anonymous=True)
        # Mic_agv()
        # rospy.spin()
        mic = Mic_agv()
        mic.spin()
    except rospy.ROSInterruptException:
        pass
