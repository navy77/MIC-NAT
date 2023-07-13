#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;
float value[]={1.1,2.2};
std_msgs::Float32MultiArray ary_msg;
ros::Publisher chatter("array", &ary_msg);

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  ary_msg.data = value;
  ary_msg.data_length = 2;
  chatter.publish( &ary_msg );
  
  nh.spinOnce();
  delay(2000);
}
