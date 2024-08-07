#include <ros.h>
#include <std_msgs/Int32.h>

int Xchannel = 25;
int Ychannel = 26;
int X_joy = 130; 
int Y_joy = 130;  

float velLinear = 0.0;
float velAngular = 0.0;

void velLinearCallback(const std_msgs::Int32& msg);
void velAngularCallback(const std_msgs::Int32& msg);

ros::Subscriber<std_msgs::Int32> sub_vel_linear("/vel_linear", &velLinearCallback);   //joy_y
ros::Subscriber<std_msgs::Int32> sub_vel_angular("/vel_angular", &velAngularCallback); //joy_x

ros::NodeHandle nh;

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(9600);
  
  Serial.begin(9600);

  pinMode(Xchannel, OUTPUT);
  pinMode(Ychannel, OUTPUT);

  dacWrite(Xchannel, X_joy);
  dacWrite(Ychannel, Y_joy);

  nh.subscribe(sub_vel_linear);
  nh.subscribe(sub_vel_angular);
}

void loop() {
  nh.spinOnce();
  delay(10); 
}

void velLinearCallback(const std_msgs::Int32& msg) {
  velLinear = msg.data;
  updateJoystick();
}

void velAngularCallback(const std_msgs::Int32& msg) {
  velAngular = msg.data;
  updateJoystick();
}

void updateJoystick() {
  Y_joy = velLinear;
  X_joy = velAngular;

  dacWrite(Xchannel, X_joy);
  dacWrite(Ychannel, Y_joy);
}

void writeJoystickManually() {
  if (Serial.available() > 0) {  // No Line Ending

    X_joy = Serial.parseFloat();
    Y_joy = Serial.parseFloat();

    Serial.println("X: " + String(X_joy));
    Serial.println("Y: " + String(Y_joy));

    dacWrite(Xchannel, X_joy);
    dacWrite(Ychannel, Y_joy);
  }
}
