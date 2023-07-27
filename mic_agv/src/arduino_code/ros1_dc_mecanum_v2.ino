
// ##################################################################//
//  Ros version : Noetic
//  Drive type: Mechanum 4wd
//  Date : 12/6/2023
//  Rev. : 2
// ##################################################################//
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
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
const uint8_t DIR[] = {4, 46, 6, 42};
const uint8_t PWM[] = {5, 45, 7, 39};
const uint8_t EA[] = {8, 37, 10, 35};
const uint8_t EB[] = {9, 38, 11, 36};

long int enc_count_0;
long int enc_count_1;
long int enc_count_2;
long int enc_count_3;

long prv_time[2];

const uint8_t LED[] = {19, 20, 21}; // R,G,B

// Motor
int pwm_value[4];
int dir_motor[4];

// IR sensor
const uint8_t IR[] = {13, 14};

// ##################################################################//
ros::NodeHandle nh;
// Pub IMU data
std_msgs::Float32 imu_msg;
ros::Publisher IMU("imu_data", &imu_msg);

// Pub Encoder data
std_msgs::Int32MultiArray enc_msg;
ros::Publisher ENC("enc_data", &enc_msg);

// Pub voltage data
std_msgs::Float32 vol_msg;
ros::Publisher VOL("vol_data", &vol_msg);

// Pub ir data
std_msgs::Int32MultiArray ir_msg;
ros::Publisher IR_sensor("ir_data", &ir_msg);

// Service Clear encoder result
void srv_clear_data(const Test::Request &req, Test::Response &res);

// Service
ros::ServiceServer<Test::Request, Test::Response> clear_enc_data("clear_enc_data", &srv_clear_data);
// ##################################################################//

void pin_define()
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(EA[i], INPUT_PULLUP);
    pinMode(EB[i], INPUT_PULLUP);
    pinMode(DIR[i], OUTPUT);
    pinMode(PWM[i], OUTPUT);
  }

  for (int i = 0; i < 2; i++)
  {
    pinMode(IR[i], OUTPUT);
  }

  for (int i = 0; i < 3; i++)
  {
    pinMode(LED[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}

void PWM_callback(const std_msgs::Float32MultiArray &msg)
{
  pwm_value[0] = msg.data[0];
  pwm_value[1] = msg.data[1];
  pwm_value[2] = msg.data[2];
  pwm_value[3] = msg.data[3];
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_pwm("/pwm_data", &PWM_callback);

void setup()
{
  nh.initNode();
  pin_define();

  // IMU
  // Serial.begin(57600);
  mySerial.begin(9600);
  IMU_sensor.modifyFrequency(FREQUENCY_20HZ);

  // pub
  nh.advertise(IMU);
  nh.advertise(VOL);
  nh.advertise(IR_sensor);
  nh.advertise(ENC);

  // service
  nh.advertiseService(clear_enc_data);

  // sub
  nh.subscribe(sub_pwm);
  enc_count_0 = 0;
  enc_count_1 = 0;
  enc_count_2 = 0;
  enc_count_3 = 0;
}

void loop()
{
  if (millis() - prv_time[0] > 50)
  {
    enc_array(enc_count_0, enc_count_1, enc_count_2, enc_count_3);
    // IMU_dfrobot();
    agv_run();
    prv_time[0] = millis();
  }
  nh.spinOnce();
}
void agv_run(){
  digitalWrite(DIR[0], LOW);
  analogWrite(PWM[0], abs(pwm_value[0]));
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

void ir_array()
{
  int ir_0 = analogRead(IR[0] * 0.004882813); // 5/1024
  int ir_1 = analogRead(IR[1] * 0.004882813); // 5/1024

  int ir_value[] = {ir_0, ir_1};
  ir_msg.data = ir_value;
  ir_msg.data_length = 2;
  IR_sensor.publish(&ir_msg);
}

void enc_array(int e0, int e1, int e2, int e3)
{
  int enc_value[] = {e0, e1, e2, e3};
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
