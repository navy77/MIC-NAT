
// ##################################################################//
//  Ros version : Noetic
//  Drive type: Mechanum 4wd
//  Date : 12/8/2023
// ##################################################################//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;
#include <DFRobot_WT61PC.h>
#include "SoftwareSerial.h"
SoftwareSerial mySerial(17, 18); // (RX,TX)
DFRobot_WT61PC IMU_sensor(&mySerial);
// ##################################################################//
// Motor control --> [0]->front-l , [1]->front-r ,[2]->rear-l ,[3]->rear-r
const uint8_t DIR[] = {7, 42, 5, 38};
const uint8_t PWM[] = {6, 39, 4, 37};
const uint8_t EA[] = {10,36,13,33};
const uint8_t EB[] = {11,35,12,34};
// IR sensor 
const uint8_t ir_sensor[] = {14, 15};
// LED
const uint8_t LED[] = {19, 20, 21}; // R,G,B

// ##################################################################//
// Enc parameter
long enc_count_0;
long enc_count_1;
long enc_count_2;
long enc_count_3;

// Time parameter
unsigned long prv_time[2];

// Motor parameter
int16_t pwm_value[4];
uint8_t dir[4];

// CMD vel
float d_time;
const int TPM[] = {902,1003}; //tick/meter;
const int pub_rate = 50;//Hz
float lx = 0.3; //m
float ly = 0.3; //m
float wheel_diameter = 0.127; //m
float vel_act[4];
float vel_flt[4];
float vel_prv[4];
float distance[4];
int enc_value[4];
// PID Control

// ##################################################################//

ros::NodeHandle nh;
// Pub IMU data
std_msgs::Float32 imu_msg;
ros::Publisher IMU("imu_data", &imu_msg);

// Pub Encoder data
std_msgs::Int32MultiArray enc_msg;
ros::Publisher ENC("enc_data", &enc_msg);

// Pub ir data
std_msgs::Int32MultiArray ir_msg;
ros::Publisher IR_sensor("ir_data", &ir_msg);

// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);
// ##################################################################//
void gpio_define()
{   
    for(int i=0;i<4;i++){
      pinMode(EA[i],INPUT_PULLUP);
      pinMode(EB[i],INPUT_PULLUP);
      pinMode(DIR[i], OUTPUT);
      pinMode(PWM[i], OUTPUT);
    }

    for(int i=0;i<3;i++){
      pinMode(LED[i], OUTPUT);
    }
    
    for(int i=0;i<2;i++){
      pinMode(ir_sensor[i], OUTPUT);
    }
    attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}
void CMD_vel_callback(const geometry_msgs::Twist &CVel){
  
}

ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMD_vel_callback);

void setup() {
  nh.initNode();
  gpio_define();
  // IMU
  mySerial.begin(9600);
  IMU_sensor.modifyFrequency(FREQUENCY_20HZ);

  //Pub
  nh.advertise(IMU);
  nh.advertise(ENC);
  nh.advertise(IR_sensor);
  //Sub
  nh.subscribe(sub_cmdVel);
  // service
  nh.advertiseService(clear_enc_data);

  enc_count_0 = 0;
  enc_count_1 = 0;
  enc_count_2 = 0;
  enc_count_3 = 0;
}

void loop() {
  int pwm = 10;
  int dir = 1;
  if (millis()-prv_time[0]>=50){
      d_time = millis()-prv_time[0];
//      IMU_dfrobot();
      enc2array(enc_count_0, enc_count_1, enc_count_2, enc_count_3);
      vel_calculate(enc_value,4);
      
      prv_time[0] = millis();
  }
  
//  agv_run(pwm,dir);
  nh.spinOnce();
}

void agv_run(int pwm_,int dir_){
  digitalWrite(DIR[0], dir_);
  analogWrite(PWM[0], pwm_);
}

void IMU_dfrobot()
{
  if (IMU_sensor.available())
  {
    if (IMU_sensor.Gyro.Z > 250)
    {
      imu_msg.data = (500 - IMU_sensor.Gyro.Z) * 0.017453; // unit rad/s
      IMU.publish(&imu_msg);
    }
    else
    {
      imu_msg.data = -0.017453 * IMU_sensor.Gyro.Z;
      IMU.publish(&imu_msg);
    }
  }
}

void vel_calculate(int enc_array[],int n){
      imu_msg.data = enc_array[0];
      IMU.publish(&imu_msg);
  
}

void enc2array(int e0, int e1, int e2, int e3)
{
  enc_value[0] = e0;
  enc_value[1] = e1;
  enc_value[2] = e2;
  enc_value[3] = e3;
  
  int enc_value[4] = {e0, e1, e2, e3};
  enc_msg.data = enc_value;
  enc_msg.data_length = 4;
  ENC.publish(&enc_msg);
}

void srv_clear_data(const Test::Request &req, Test::Response &res)
{
  enc_count_0 = 0;
  enc_count_1 = 0;
  enc_count_2 = 0;
  enc_count_3 = 0;
  res.output = "Data has been cleared ";
}

void enc_count0()
{
  if (digitalRead(EB[0]) == 0)
  {
    if (digitalRead(EA[0]) == 0)
    {
      enc_count_0++;
    }
    else
    {
      enc_count_0--;
    }
  }
}

void enc_count1()
{
  if (digitalRead(EB[1]) == 0)
  {
    if (digitalRead(EA[1]) == 0)
    {
      enc_count_1++;
    }
    else
    {
      enc_count_1--;
    }
  }
}

void enc_count2()
{
  if (digitalRead(EB[2]) == 0)
  {
    if (digitalRead(EA[2]) == 0)
    {
      enc_count_2++;
    }
    else
    {
      enc_count_2--;
    }
  }
}

void enc_count3()
{
  if (digitalRead(EB[3]) == 0)
  {
    if (digitalRead(EA[3]) == 0)
    {
      enc_count_3++;
    }
    else
    {
      enc_count_3--;
    }
  }
}
