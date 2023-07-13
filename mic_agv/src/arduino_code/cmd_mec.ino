#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Wire.h>
#include <rosserial_arduino/Test.h>
using rosserial_arduino::Test;

const uint8_t DIR[] = {4,46,6,42};
const uint8_t PWM[] = {5,45,7,39};
const uint8_t EA[] = {8,37,10,35};
const uint8_t EB[] = {9,38,11,36};

long int enc_count[4];
long int enc_count_0;
long int enc_count_1;
long int enc_count_2;
long int enc_count_3;
long int prv_enc_count[4];
const int TPM[] = {2835,3150};//tick/meter
float lx = 0.3; //m
float ly = 0.3; //m
float wheel_diameter = 0.127; //m
long prv_time[2];
float distance[4];
float d_time;
float vel_x,vel_y,vel_z;
int DIR_[4]; //motor direction
float vel[4];
float vel_act[4];
float vel_flt[4];
float vel_prv[4];
float vel_trg[4];

float prv_err[4];
float err[4];
float I_err[4];
float D_err[4];
//float KP = 50;
//float KI = 10;
//float KD = 5;
int pwm_value[4];

ros::NodeHandle nh;
// Pub vel data
std_msgs::Float32 vel_msg_T;
ros::Publisher VEL_T("vel_T", &vel_msg_T);

std_msgs::Float32 vel_msg_0;
ros::Publisher VEL_0("vel_value_0", &vel_msg_0);

std_msgs::Float32 vel_msg_1;
ros::Publisher VEL_1("vel_value_1", &vel_msg_1);

std_msgs::Float32 vel_msg_2;
ros::Publisher VEL_2("vel_value_2", &vel_msg_2);

std_msgs::Float32 vel_msg_3;
ros::Publisher VEL_3("vel_value_3", &vel_msg_3);

// Pub pwm data
std_msgs::Float32 pwm_msg_0;
ros::Publisher PWM_0("pwm_value_0", &pwm_msg_0);

std_msgs::Float32 pwm_msg_1;
ros::Publisher PWM_1("pwm_value_1", &pwm_msg_1);

std_msgs::Float32 pwm_msg_2;
ros::Publisher PWM_2("pwm_value_2", &pwm_msg_2);

std_msgs::Float32 pwm_msg_3;
ros::Publisher PWM_3("pwm_value_3", &pwm_msg_3);

void CMD_vel_callback(const geometry_msgs::Twist &CVel)
{
  vel_x = CVel.linear.x;
  vel_y = CVel.linear.y;
  vel_z = CVel.angular.z;

  if (vel_x == 0 && vel_z == 0) //stop
  {
    for(int i=0;i<4;i++ ){
      vel[i] = 0.0;
      analogWrite(PWM[i],0);
      pid_clear();
    }
  }
  else
  {
       vel[0] = (vel_x-vel_y-(lx+ly)*vel_z); //w-fl
       vel[1] = (vel_x+vel_y+(lx+ly)*vel_z); //w-fr
       vel[2] = (vel_x+vel_y-(lx+ly)*vel_z); //w-rl
       vel[3] = (vel_x-vel_y+(lx+ly)*vel_z); //w-rr
       
       for(int i=0;i<4;i++ ){
        DIR_[i] = vel[i]/abs(vel[i]);
            if(DIR_[i]>0){
              DIR_[i] = 1;
            }
            else{
              DIR_[i] = 0;
            }
       }
  }
  
}
ros::Subscriber<geometry_msgs::Twist> sub_cmdVel("/cmd_vel", &CMD_vel_callback);

void setup() {
    nh.initNode();
    nh.advertise(VEL_0);
    nh.advertise(VEL_1);
    nh.advertise(VEL_2);
    nh.advertise(VEL_3);
    
    nh.advertise(VEL_T);
    
    nh.advertise(PWM_0);
    nh.advertise(PWM_1);
    nh.advertise(PWM_2);
    nh.advertise(PWM_3);
    
    nh.subscribe(sub_cmdVel);
    
    for(int i=0;i<4;i++){
      pinMode(EA[i],INPUT_PULLUP);
      pinMode(EB[i],INPUT_PULLUP);
      pinMode(DIR[i],OUTPUT);
      pinMode(PWM[i],OUTPUT);
    }
    attachInterrupt(digitalPinToInterrupt(EA[0]), enc_count0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[1]), enc_count1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[2]), enc_count2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EA[3]), enc_count3, CHANGE);
}

void loop() {
  nh.spinOnce();
    if (millis()-prv_time[0]>50){

      pid(2,2,1,5,250,-250);
      prv_time[0] = millis();
     }
  agv_run();
}

void vel_calculate(int enc0,int enc1,int enc2,int enc3){
  enc_count[0]= enc0;
  enc_count[1]= enc1;
  enc_count[2]= enc2;
  enc_count[3]= enc3;
  
  if(millis()-prv_time[0]>50) {
    d_time = millis() - prv_time[0];
  
    for(int i=0;i<2;i++){
      distance[i] = float(enc_count[i]-prv_enc_count[i])/TPM[0];
      vel_act[i] = (distance[i]/d_time)*1000;
    }

    for(int i=2;i<4;i++){
      distance[i] = float(enc_count[i]-prv_enc_count[i])/TPM[1];
      vel_act[i] = (distance[i]/d_time)*1000;
    }
    // Filter velocity at 25 Hz
    for(int i=0;i<4;i++){
      vel_flt[i] = 0.854*vel_flt[i]+0.0728*vel_act[i]+0.0728*vel_prv[i];
      vel_prv[i] = vel_flt[i];
    }
    for(int i=0;i<4;i++){
      prv_enc_count[i] = float( enc_count[i]);
    }

    prv_time[0] = millis();
    
    vel_msg_0.data = vel_flt[0];
    vel_msg_1.data = vel_flt[1];
    vel_msg_2.data = vel_flt[2];
    vel_msg_3.data = vel_flt[3];
  
    VEL_0.publish(&vel_msg_0);
    VEL_1.publish(&vel_msg_1);
    VEL_2.publish(&vel_msg_2);
    VEL_3.publish(&vel_msg_3);
   }
  }

void pid(float KP,float KI,float KD,float wind_err,int max_pwm,int min_pwm){
  vel_calculate(enc_count_0,enc_count_1,enc_count_2,enc_count_3);
  
  vel_msg_T.data = vel[0];
  VEL_T.publish(&vel_msg_T);
  
  vel_trg[0]=vel[0];
  
  err[0] = vel_trg[0] - vel_flt[0];
  I_err[0] = I_err[0] + err[0];
  D_err[0] = err[0] - prv_err[0];

  if(vel_trg[0] == 0 ){
    I_err[0] = 0;
  }

  if(abs(I_err[0])> wind_err){ 
    I_err[0] =I_err[0]*0.5;
  }
  
  pwm_value[0] = KP*err[0]+KI*I_err[0]+KD*D_err[0];
  prv_err[0] = err[0];

  if(pwm_value[0]> max_pwm){
    pwm_value[0]= max_pwm;
  }
  if(pwm_value[0]< min_pwm){
    pwm_value[0]= min_pwm;
  }
  
  pwm_msg_0.data = pwm_value[0];
  PWM_0.publish(&pwm_msg_0);

}
void agv_run(){

  digitalWrite(DIR[0], LOW);
  analogWrite(PWM[0],abs(pwm_value[0]));
}
void pid_clear()
{
  for(int i=0;i<4;i++){
    prv_err[i]=0;
    err[i]=0;
    I_err[i]=0;
    D_err[i]=0;
  }
}

void enc_count0(){
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

void enc_count1(){
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

void enc_count2(){
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

void enc_count3(){
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
