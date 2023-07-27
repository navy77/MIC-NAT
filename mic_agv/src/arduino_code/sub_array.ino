#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;
int value[2];
long prv_time;
std_msgs::Float32MultiArray ary_msg;

void array_callback(const std_msgs::Float32MultiArray &msg)
{
  value[0] = msg.data[0];
  value[1] = msg.data[1];
  //value = msg.data;
}

ros::Subscriber<std_msgs::Float32MultiArray> s("pub_array", &array_callback);

void setup()
{
  nh.initNode();
  nh.subscribe(s);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
}

void loop()
{ 
  if(millis()-prv_time >100){
    if(value[0] > 1){
      digitalWrite(LED_BUILTIN, HIGH); 
    }
//    digitalWrite(LED_BUILTIN, LOW);
    prv_time = millis();
  }
  
  nh.spinOnce();
}
