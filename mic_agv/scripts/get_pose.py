#!/usr/bin/env python3
import rospy
import roslib
import csv
import math
from std_msgs.msg import Int32,Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
# MQTT
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
host = '192.168.1.101'
port = 1883
topic = '/ros_mqtt'

class Semi_movement():
    def __init__(self):
        self.move_distance = Float64()
        self.move_distance.data = 0
        self.get_init_pose()

        self.distance_pub = rospy.Publisher("/move_distance",Float64,queue_size=1)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)
    
    def get_init_pose(self):
        data_odom = None
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odom",Odometry,timeout=1)
            except:
                rospy.loginfo("Odom not ready yet!!!")
        
        self.current_position = Point()
        self.current_position.x = data_odom.pose.pose.position.x
        self.current_position.y = data_odom.pose.pose.position.y
        self.current_position.z = data_odom.pose.pose.position.z
    
    def odom_callback(self,msg):
        new_position = msg.pose.pose.position
        self.move_distance.data = self.calculate_distance(new_position,self.current_position)
        self.update_current_position(new_position)
        if self.move_distance.data <0.00001:
            aux = Float64()
            aux.data = 0.0
            self.distance_pub.publish(aux)
        else:
            self.distance_pub.publish(self.move_distance)
    
    def update_current_position(self,new_position):
        self.current_position.x = new_position.x
        self.current_position.y = new_position.y
        self.current_position.z = new_position.z

    def calculate_distance(self,new_position,old_position):
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        dist = math.hypot(x2-x1,y2-y1)
        return dist
  