#include <Wire.h> 
#include <MechaQMC5883.h> 
#include <ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

MechaQMC5883 bussola;

int x = 0, y = 0, z = 0;
float angulo = 0;
float mag_declination = 16.03;

std_msgs::Float32 compass_msg;
ros::Publisher compass_pub("compass_data", &compass_msg);

ros::NodeHandle nh;

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  Wire.begin(); 
  Serial.begin(9600); 
  
  bussola.init();
  
  nh.advertise(compass_pub);
}
 
void loop() { 
  bussola.read(&x, &y, &z); 
  angulo = atan2(x, y) / (180.0/M_PI);
  angulo = angulo - mag_declination;

  if(angulo < 0){
    angulo += 360;
  }
 
  compass_msg.data = angulo;
  compass_pub.publish(&compass_msg);

  nh.spinOnce();
  
  delay(500);
}
