#include <ros.h>
#include "ArduinoJson.h"

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// Variable
float steerSpeed = 0;
float targetSpeed = 0;
float currentSpeed = 0;
DynamicJsonDocument doc(100);
String jsonString = "";

// Ros
void messageTargetSpd(const geometry_msgs::Twist& msg);
geometry_msgs::Twist msgs;

ros::NodeHandle nh;
std_msgs::String str_msg;

// ros::Publisher pub("arduino_speed_out", &str_msg);


void messageTargetSpd(const geometry_msgs::Twist& msg) {
  // str_msg.data = "publisher initiallized";
  // pub.publish( &str_msg );
  targetSpeed = (float)msg.linear.x;
  currentSpeed = (float)msg.linear.z;
  steerSpeed = (float)msg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageTargetSpd);


void setup() {


  Serial.begin(57600); // ros - rosArduino 
  Serial1.begin(115200); // TX18 RX19 rosArduino - velocity 
  Serial2.begin(38400); // TX16 RX17 rosArduino - steer

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  // nh.advertise(pub);

  
}

void loop() {

  // 종방향 메인으로 보내줌
  if(Serial1.available()){
    // jsonString = "{\"cur\":\"" + String(currentSpeed) +"\",\"tar\":\"" + String(targetSpeed) + "\"}";
    // doc["cur"] = String(currentSpeed);
    // doc["tar"] = String(targetSpeed);
    // serializeJson(doc, jsonString);
    // doc.clear();
    // Serial1.println(jsonString);
    Serial1.println(String(targetSpeed));
    delay(333);

  }

  // 횡방향 슬래이브로 보내줌
  if(Serial2.available()){
    Serial2.println(String(steerSpeed));
  }

  nh.spinOnce();


}