#!/usr/bin/env python3
import rospy
from math import sin,cos,pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32,Float32,Int32MultiArray,Float32MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class Mic_ros:
    def __init__(self):
        rospy.init_node("agv_mecanum",anonymous=True)
        self.rate = 2
        self.enc_value = rospy.Subscriber('/enc_data',Int32MultiArray,self.cb_enc_value)
        self.vel_value = rospy.Subscriber('/vel_data',Float32MultiArray,self.cb_vel_value)
        self.dis_value = rospy.Subscriber('/dis_data',Float32MultiArray,self.cb_dis_value)
        self.imu_value = rospy.Subscriber('/imu_data',Float32,self.cb_imu_value)
        
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
        # mecanum wheel
        self.lx = 0.3
        self.ly = 0.3
        self.wheel_diameter = 0.127
        self.encoder_tick_0,self.encoder_tick_1 = 902,1003
        # encoder data
        self.enc_0 = 0
        self.enc_1 = 0
        self.enc_2 = 0
        self.enc_3 = 0

        self.prv_enc_0 = 0
        self.prv_enc_1 = 0
        self.prv_enc_2 = 0
        self.prv_enc_3 = 0

        self.vel_0 = 0
        self.vel_1 = 0
        self.vel_2 = 0
        self.vel_3 = 0

        self.dis_0 = 0
        self.dis_1 = 0
        self.dis_2 = 0
        self.dis_3 = 0

        self.th = 0 
        self.imu = 0 # rad/s
        # odom
        self.th = 0 
        self.x = 0
        self.y = 0
        self.base_frame_id = "base_link"
        self.odom_frame_id = "odom"

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()

        self.pos_x  = 0
        self.pos_y  = 0

        self.vel_x = 0
        self.vel_y = 0
        self.vel_z = 0
    
    def cb_enc_value(self,msg):
        self.enc_0 = msg.data[0]
        self.enc_1 = msg.data[1]
        self.enc_2 = msg.data[2]
        self.enc_3 = msg.data[3]

    def cb_vel_value(self,msg):
        self.vel_0 = msg.data[0]
        self.vel_1 = msg.data[1]
        self.vel_2 = msg.data[2]
        self.vel_3 = msg.data[3]

    def cb_dis_value(self,msg):
        self.dis_0 = msg.data[0]
        self.dis_1 = msg.data[1]
        self.dis_2 = msg.data[2]
        self.dis_3 = msg.data[3]

    def cb_imu_value(self,msg):
        self.imu = msg.data # unit rad/s

    def update(self):
        
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if self.enc_0 == None:
                d_0 = 0
                d_1 = 0
                d_2 = 0
                d_3 = 0
            else:
                # d_0 = (self.enc_0-self.prv_enc_0)/self.encoder_tick_0
                # d_1 = (self.enc_1-self.prv_enc_1)/self.encoder_tick_0
                # d_2 = (self.enc_2-self.prv_enc_2)/self.encoder_tick_1
                # d_3 = (self.enc_3-self.prv_enc_3)/self.encoder_tick_1
                d_0 = self.dis_0
                d_1 = self.dis_1
                d_2 = self.dis_2
                d_3 = self.dis_3

            self.prv_enc_0 = self.enc_0
            self.prv_enc_1 = self.enc_1
            self.prv_enc_2 = self.enc_2
            self.prv_enc_3 = self.enc_3

            th = self.imu
            
            deltaXTravel = (d_0 + d_1 + d_2 + d_3)/4.0
            deltaYTravel = (-d_0 + d_1 + d_2 - d_3)/4.0

            self.vel_x = (self.dis_0+self.dis_1+self.dis_2+self.dis_3)/4
            self.vel_y = (-self.dis_0+self.dis_1+self.dis_2-self.dis_3)/4

            self.x += deltaXTravel*cos(th) - deltaYTravel*sin(th)
            self.y += deltaYTravel*cos(th) + deltaXTravel*sin(th)
            self.th += th 

            # deltaXTravel = (frontLeftTravel + frontRightTravel + rearLeftTravel + rearRightTravel) / 4.0
            # deltaYTravel = (-frontLeftTravel + frontRightTravel + rearLeftTravel - rearRightTravel) / 4.0
            # deltaTheta = (-frontLeftTravel + frontRightTravel - rearLeftTravel + rearRightTravel) / (2 * (self.wheelSeparation + self.wheelSeparationLength))
            print(deltaXTravel,"::",deltaYTravel,"::",th,"::",self.x,"::",self.y,"::",self.th)
            
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th/2)
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x,self.y,0),
                (quaternion.x,quaternion.y,quaternion.z,quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.child_frame_id = self.base_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion

            odom.twist.twist.linear.x = self.vel_x
            odom.twist.twist.linear.y = self.vel_y
            odom.twist.twist.angular.z = self.imu
            self.odomPub.publish(odom)

    def spin(self):
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    try:
        mic = Mic_ros()
        mic.spin()
    except rospy.ROSInterruptException:
        pass
