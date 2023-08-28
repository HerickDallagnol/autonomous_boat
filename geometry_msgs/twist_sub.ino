#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
geometry_msgs::Twist msg;

float move1;
float move2;

void move(const geometry_msgs::Twist &cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel",move);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(25,OUTPUT);
  pinMode(26,OUTPUT);

} 

void loop()
{
  dacWrite(25,move1);
  dacWrite(26,move2);
  nh.spinOnce();
}
