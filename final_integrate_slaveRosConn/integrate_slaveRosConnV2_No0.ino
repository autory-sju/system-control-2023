#include <ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// Variable
float steerSpeed = 0;
float targetSpeed = 0;
float currentSpeed = 0;
float speedDis = 0;
String speedDisString = "";
String steerSpeedString = "";
unsigned long timeTmp = 0;


int i = 0;

unsigned long t = 0;

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


  Serial.begin(57600);    // ros - rosArduino
  Serial1.begin(115200);  // TX18 RX19 rosArduino - velocity
  Serial2.begin(38400);   // TX16 RX17 rosArduino - steer

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  // nh.advertise(pub);
}

void loop() {


  // 종방향 마스터로 보내줌
  speedDis = targetSpeed * 100 - currentSpeed * 100;

  speedDisString = "s" + String(speedDis) + "f";
  if (millis() - timeTmp > 2000) {
    for (i = 0; i < speedDisString.length(); i++) {
      Serial1.write(speedDisString[i]);
    }
    timeTmp = millis();
  }




  // if (millis() - t > 1000) {

  // 횡방향 슬레이브로 전송
  if (steerSpeed != 0) {
    steerSpeedString = "s" + String((int)(steerSpeed * 100)) + "f";
    for (i = 0; i < steerSpeedString.length(); i++) {
      Serial2.write(steerSpeedString[i]);
    }
  }
  steerSpeed = 0;
  speedDis = 0;  // 무지성 작동 방지 코드
  //뭔가 이상하면 0일 때 보내지 말거나 지우자


  //   t = millis();
  // }
  nh.spinOnce();
  delay(50);
}
