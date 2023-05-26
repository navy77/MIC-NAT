#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import paho.mqtt.subscribe as subscribe
import rospy
from std_msgs.msg import Int32,String
from geometry_msgs.msg import Twist
from time import *

class Agv_api:

    def __init__(self):
        self.host = '192.168.1.101'
        self.port = 1883
        self.client = mqtt.Client()
        self.client.connect(self.host,self.port)
        self.tpm = 1698
        self.encoder1_topic = '/enc_1_value'
        self.encoder2_topic = '/enc_2_value'
        self.detect1_topic = '/ir_1_value'
        self.detect2_topic = '/ir_2_value'
        self.robot_loc_topic = '/robot_loc'

    def mqtt_sub(self,topic):
        self.msg = subscribe.simple(topics=str(topic),hostname=self.host,port=self.port)
        self.msg_payload = self.msg.payload.decode("utf-8", "strict")
        return self.msg_payload
    
    def mqtt_pub(self,topic,msg):
        self.client.publish(topic,msg)

    def move_straight(self,distance,vel_x): # input distance[m] with direction
        self.target_tick = self.tpm * abs(distance)
        rospy.Subscriber(self.enc1_topic,Int32,self.)
    
    def subscribe_topic():
