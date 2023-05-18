#!/usr/bin/env python3

import csv
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
host = '192.168.1.250'
port = 1883
topic = '/ros_mqtt'

def mqtt():
    global msg_payload
    msg = subscribe.simple(topics=str(topic), hostname=host,port=port)
    msg_payload = msg.payload.decode("utf-8", "strict")
   # print(msg_payload)

def read_csv():
    thisdict = {}
    with open("/home/mic/catkin_ws/src/mic_agv/scripts/location.csv", "r") as csv_file:            
        csv_reader = csv.reader(csv_file,delimiter = ',')            
        for row in csv_reader:
            print(row)
            thisdict[row[0]] = [row[1], row[2], row[3]]
    return thisdict

def read_position():  
    dict_position = {}
    dict_position = read_csv()
    print(dict_position[msg_payload])
    return dict_position[msg_payload]


if __name__ == '__main__':
    try:
       # read_csv()
        mqtt()
        read_position()
        #print(msg_payload)
    except :
        print("error")
