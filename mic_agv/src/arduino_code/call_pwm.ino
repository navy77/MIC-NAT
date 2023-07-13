#include <ros.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;

const uint8_t DIR[] = {4,46,6,42};
const uint8_t PWM[] = {5,45,7,39};
long prv_time[1];
int pwm;
ros::NodeHandle nh;

void pwm_callback(const std_msgs::Int32 &PWM_value)
{
  pwm = PWM_value.data;
}
  
ros::Subscriber<std_msgs::Int32> sub_pwm("/pwm", &pwm_callback);

void setup() {
    nh.initNode();  
    nh.subscribe(sub_pwm);
    
    for(int i=0;i<4;i++){
      pinMode(DIR[i],OUTPUT);
      pinMode(PWM[i],OUTPUT);
    }
}

void loop() {
  nh.spinOnce();
    if (millis()-prv_time[0]>100){
      agv_run();
      prv_time[0] = millis();
     }
  
}

void agv_run(){
  digitalWrite(DIR[0], LOW);
  analogWrite(PWM[0],pwm);
}
