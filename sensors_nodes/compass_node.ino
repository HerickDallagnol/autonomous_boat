#include <Wire.h> 
#include <MechaQMC5883.h> 
#include <ros.h>
#include <std_msgs/String.h>

MechaQMC5883 bussola; // Criacao do objeto para o sensor 

int x = 0, y = 0, z = 0;
int angulo = 0;

std_msgs::String compass_msg;
ros::Publisher compass_pub("compass_data", &compass_msg);

ros::NodeHandle nh;

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  Wire.begin(); 
  Serial.begin(9600); 
  
  bussola.init(); // Inicializando o Sensor QMC5883
  
  nh.advertise(compass_pub);
}
 
void loop() 
{ 
  bussola.read(&x, &y, &z); 
  angulo = atan2(x, y) / 0.0174532925; 
 
  // Ajuste do angulo entre 0 e 360 graus
  if (angulo < 0) 
    angulo += 360;
  
  angulo = 360 - angulo;
  String direction = getCompassDirection(angulo);

  compass_msg.data = direction.c_str();
  compass_pub.publish(&compass_msg);

  nh.spinOnce();
  
  delay(500);
}

String getCompassDirection(int angle) {
  if (angle > 338 || angle < 22) {
    return "Norte";
  } else if (angle > 22 && angle < 68) {
    return "Nordeste";
  } else if (angle > 68 && angle < 113) {
    return "Leste";
  } else if (angle > 113 && angle < 158) {
    return "Suldeste";
  } else if (angle > 158 && angle < 203) {
    return "Sul";
  } else if (angle > 203 && angle < 248) {
    return "Suldeste";
  } else if (angle > 248 && angle < 293) {
    return "Oeste";
  } else if (angle > 293 && angle < 338) {
    return "Noroeste";
  }
  return "Desconhecido";
}
