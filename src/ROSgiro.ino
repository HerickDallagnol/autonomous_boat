#include <Wire.h> 
#include <MechaQMC5883.h> 
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <math.h>

MechaQMC5883 bussola;

int x = 0, y = 0, z = 0;
float angulo = 0;
float angulo1 = 0;
float mag_declination = 16.03;

std_msgs::Float32 compass_msg;
ros::Publisher compass_pub("compass_data", &compass_msg);

std_msgs::Bool north_msg;
ros::Publisher north_pub("north_detected", &north_msg);

ros::NodeHandle nh;

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  Wire.begin(); 
  Serial.begin(9600); 
  
  bussola.init();
  
  nh.advertise(compass_pub);
  nh.advertise(north_pub);
}

void loop() { 
  bussola.read(&x, &y, &z); 
  angulo = atan2(x, y) / (180.0 / M_PI);
  angulo1 = angulo - mag_declination;

  if (angulo1 < 0) {
    angulo1 += 360;
  }
  compass_msg.data = angulo1;
  compass_pub.publish(&compass_msg);

  if (angulo < 0) {
    angulo += 360;
  }
  if (fabs(angulo) < 5.0) { 
    north_msg.data = true;
  } else {
    north_msg.data = false;
  }
  north_pub.publish(&north_msg);

  nh.spinOnce();
  
  delay(500);
}
