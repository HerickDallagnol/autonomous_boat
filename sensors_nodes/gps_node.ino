#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define RXD2 6
#define TXD2 7

const int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXD2, TXD2);

std_msgs::Float32 lat_msg;
std_msgs::Float32 lng_msg;
ros::Publisher lat_pub("gps_latitude", &lat_msg);
ros::Publisher lng_pub("gps_longitude", &lng_msg);

ros::NodeHandle nh;

void setup() {

  nh.initNode();
  nh.getHardware()->setBaud(9600);
  
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
  nh.advertise(lat_pub);
  nh.advertise(lng_pub);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      lat_msg.data = gps.location.lat();
      lng_msg.data = gps.location.lng();

      lat_pub.publish(&lat_msg);
      lng_pub.publish(&lng_msg);
     
    }
  }

  nh.spinOnce();
  delay(500);
}
