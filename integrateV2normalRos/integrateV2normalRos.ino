#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include "mas001.h"
#include "blc200.h"
#include <SoftwareSerial.h>


// analong pin
const int accel_input_pin = 15;  //A0

// digital pin
//const int speed_input_pin = 20;
const int dcm_output_pin = 44;
const int forward_R_switch_pin = 37;
const int forward_D_switch_pin = 36;
const int forward_relay_output_pin = 50;
const int auto_ON_pin = 53;
const int auto_OFF_pin = 52;
const int ves_switch_pin = 41;
const int ves_output_pin = 40;
const int brake_switch_pin = 22;

const int display_rx_pin = 31;
const int display_tx_pin = 30;

const int steer_rx_pin = 33;
const int steer_tx_pin = 32;


// constant
const int MAX_PEDAL_INPUT_1023 = 880;  // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 180;       // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;
const int LINEAR_GEAR_RATIO = 250;  // max would be 192
const int LENEAR_DEVICE_ID = 0;
const int LENEAR_LIMIT_LENGTH = 13;

// variable
char buf[30];
int directionMode = 0;
bool isAuto = true;
int autoMode = 0;
int vesOn = 0;  // 0:off / 1:on
int isBrakePushed = 0;
float steerSpeed = 0;
float targetSpeed = 0;
float currentSpeed = 0;
int speedDis = 0;
int linearSpeed = 30000;
int accelValue1023 = 0;
int accelConvertedValue255 = 0;
int autoBrakeValue100 = 0;
int accelAutoPressPercent = 0;

float err_P = 0, err_I = 0, err_D = 0, err_B = 0;
float Pv = 0.02, Iv = 0.02;  // 반응 속도
float Dv = 2;                // 급격한 변화 방지 (오버슈팅 방지)
long pressLength = 0;

// Ros
void messageTargetSpd(const std_msgs::Float64& msg);
void messageCurrentSpd(const std_msgs::Float64& msg);
void messageSteer(const std_msgs::Float64& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> sub("/tractive_control", &messageTargetSpd), subcurrentspeed("/메틀렙에서 보내줄 이름", &messageCurrentSpd);
std_msgs::String str_msg;
ros::Publisher pub("arduino_speed_out", &str_msg);

void messageTargetSpd(const std_msgs::Float64& msg) {
  targetSpeed = msg.data;
  sendDisplay(String(targetSpeed), 3);
}
void messageCurrentSpd(const std_msgs::Float64& msg) {
  currentSpeed = msg.data;
  sendDisplay(String(targetSpeed), 2);
}
void messageSteer(const std_msgs::Float64& msg) {
  // currentSpeed = msg.data;
  sendDisplay("Steer: " + String(targetSpeed), 15);
}

BLC200 linearm(9600, 100);
MAS001 myShield;
SoftwareSerial HMISerial(display_rx_pin, display_tx_pin);  // RX, TX
SoftwareSerial STEERSerial(steer_rx_pin, steer_tx_pin);
void setup() {


  Serial.begin(115200);
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(subcurrentspeed);
  nh.advertise(pub);

  pinMode(auto_ON_pin, INPUT_PULLUP);
  pinMode(auto_OFF_pin, INPUT_PULLUP);
  pinMode(ves_switch_pin, INPUT_PULLUP);
  pinMode(forward_R_switch_pin, INPUT_PULLUP);
  pinMode(forward_D_switch_pin, INPUT_PULLUP);
  pinMode(brake_switch_pin, INPUT_PULLUP);


  pinMode(dcm_output_pin, OUTPUT);
  pinMode(forward_relay_output_pin, OUTPUT);
  pinMode(ves_output_pin, OUTPUT);


  HMISerial.begin(9600);
  STEERSerial.begin(38400);
  while (!Serial) {}  // 시리얼 포트가 연결될 대까지 기다림
  Serial.println("READY");


  if (linearm.get_Feedback(LENEAR_DEVICE_ID, 0xA6)) {
    linearSpeed = (uint16_t)linearm.blcData[1] << 8 | (uint16_t)linearm.blcData[2];
  } else {
    while (true) {
      Serial.println("brake error");
      sendDisplay("LIN CON ERR", 15);
      delay(1000);
      if (linearm.get_Feedback(LENEAR_DEVICE_ID, 0xA6)) {
        linearSpeed = 30000;
        break;
      }
    }
  }

  Serial.println("plz wait 3secs");
  linearm.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  Serial.println("initialized");
  linearControl(0);
  delay(3000);
}


void loop() {
  nh.spinOnce();
  autoModeSwitch();
  driveDirectionSwitch();
  vesSwitch();
  brakeSwitch();
  showingDisplay();

  if (autoMode == 1) {  //  AS - ON
                        // current speed -> ROS velocity
                        // target speed -> ROS

    
    // targetSpeed = (int)Serial.readStringUntil('\n').toInt();
    sendDisplay(String(targetSpeed), 3);
    speedDis = targetSpeed - currentSpeed;
    speedDis = boxingInt(-5, 10, speedDis);
    
    // 횡방향
    STEERSerial.println(steerSpeed);

    // 종방향
    if (speedDis >= -1) {
      autoAcceleration();  //0%(=30)<= speedDis <= 100%(100) MAX65%
    } else if (-2 > speedDis) {
      linearControl(50);  //0%(=25)<= speedDis <= 100%(50)
    }
  } else if (autoMode == 2) {  // AS - EM
    linearControl(100);
  } else if (autoMode == 3) {  // AS - OFF
    manualAcceleration();
  }
}


void autoAcceleration() {
  err_P = speedDis;
  err_I += err_P;
  err_D = err_B - err_P;
  err_B = err_P;
  accelAutoPressPercent = ((err_P * Pv) + (err_I * Iv) + (err_D * Dv));
  accelAutoPressPercent = boxingInt(0, 100, accelAutoPressPercent);
  dcmControl(accelAutoPressPercent);
}

void manualAcceleration() {
  linearControl(0);

  accelValue1023 = analogRead(accel_input_pin);  // read analog accel input
  if (directionMode != 2) {
    dcmControl(map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 100));
  }
}



// MODE SETTING
void vesSwitch() {
  // int prevMode = isBrakePushed;

  if (digitalRead(ves_switch_pin) == 1) {
    isBrakePushed = 1;
    digitalWrite(ves_output_pin, HIGH);
  } else {
    isBrakePushed = 0;
    digitalWrite(ves_output_pin, LOW);
  }
}
void brakeSwitch() {
  if (digitalRead(brake_switch_pin) == 1) {
    isBrakePushed = 1;
  } else {
    isBrakePushed = 0;
  }
}
void autoModeSwitch() {
  int prevMode = autoMode;
  if (digitalRead(auto_ON_pin) == 0 && digitalRead(auto_OFF_pin) == 1) {  // AS ON
    isAuto = true;
    autoMode = 1;
  } else if (digitalRead(auto_ON_pin) == 1 && digitalRead(auto_OFF_pin) == 0) {  // AS OFF
    isAuto = false;
    autoMode = 3;
  } else {  // AS EM
    isAuto = false;
    autoMode = 2;
  }

  if (prevMode != autoMode) {
    analogWrite(dcm_output_pin, 0);
  }
}
void driveDirectionSwitch() {
  int prevMode = directionMode;

  if (digitalRead(forward_R_switch_pin) == 0 && digitalRead(forward_D_switch_pin) == 1) {
    directionMode = 1;
    digitalWrite(forward_relay_output_pin, HIGH);
  } else if (digitalRead(forward_R_switch_pin) == 1 && digitalRead(forward_D_switch_pin) == 0) {
    directionMode = 3;
  } else {
    directionMode = 2;
    digitalWrite(forward_relay_output_pin, LOW);
  }
  if (prevMode != directionMode) {
    analogWrite(dcm_output_pin, 0);
  }
}

// MOTOR CONTROL IN 0~100%
void dcmControl(int percent) {
  percent = boxingInt(0, 100, percent);
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);

  if (isBrakePushed) {
    analogWrite(dcm_output_pin, 0);
  } else {
    percent = boxingInt(0, 100, percent);
    accelConvertedValue255 = map(percent, 0, 100, 0, 255);  // max 150 out of 255
    analogWrite(dcm_output_pin, accelConvertedValue255);
  }

  // 지워도됨
  // percent = boxingInt(0, 100, percent);
  // accelConvertedValue255 = map(percent, 0, 100, 0, 100);  // max 150 out of 255
  // analogWrite(dcm_output_pin, accelConvertedValue255);
}

// LINEAR BRAKE CONTROL IN 0~100%
void linearControl(int percent) {
  autoBrakeValue100 = percent;
  // dcm stop
  analogWrite(dcm_output_pin, 0);

  percent = boxingInt(0, 100, percent);
  pressLength = 0;
  if (percent >= 80) pressLength = LENEAR_LIMIT_LENGTH;  // in mm
  else if (percent >= 40) pressLength = 8;
  else if (percent >= 10) pressLength = 5;
  else pressLength = 0;
  pressLength = pressLength * (65473) / 100;

  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, pressLength, linearSpeed * 10);
}



// DISPLAY
void sendDisplay(String sendData, int mode) {
  String target;
  switch (mode) {
    case 2:
      target = "t2.txt=\"";  // speed
      break;
    case 3:
      target = "t3.txt=\"";  // target
      break;
    case 5:
      target = "t5.txt=\"";  // R N D
      break;
    case 4:
      target = "t4.txt=\"";  // Manual / auto
      break;
    case 7:
      target = "t7.txt=\"";  // foot accel
      break;
    case 11:
      target = "t11.txt=\"";  // auto brake
      break;
    case 9:
      target = "t9.txt=\"";  // accel
      break;
    case 13:
      target = "t13.txt=\"";  // brake by sensor
      break;
    case 12:
      target = "t12.txt=\"";  // VES
      break;
    case 14:
      target = "t14.txt=\"";  // AS MODE
      break;
    case 15:
      target = "t15.txt=\"";  // error
      break;
    default:
      return;
  }
  target.concat(sendData);
  target.concat("\"");
  //Serial.print(mode);
  //Serial.println(sendData);
  for (int i = 0; i < target.length(); i++) {
    HMISerial.write(target[i]);  // send each char
  }
  HMISerial.write(0xff);
  HMISerial.write(0xff);
  HMISerial.write(0xff);  // end signal byte
}

void showingDisplay() {

  if (vesOn == 1) sendDisplay("VES ON", 12);
  else if (vesOn == 2) sendDisplay("VES OFF", 12);
  else sendDisplay("ERR", 12);

  if (isBrakePushed == 1) sendDisplay("BRAKE !", 11);
  else if (isBrakePushed == 0) sendDisplay("", 11);
  else sendDisplay("ERR", 11);

  if (autoMode == 1) sendDisplay("AS-ON", 14);
  else if (autoMode == 2) sendDisplay("AS-EMER", 14);
  else if (autoMode == 3) sendDisplay("MANUAL", 14);
  else sendDisplay("ERR", 14);

  if (directionMode == 1) sendDisplay("R", 5);
  else if (directionMode == 2) sendDisplay("N", 5);
  else if (directionMode == 3) sendDisplay("D", 5);
  else sendDisplay("ERR", 5);

  sendDisplay(String(currentSpeed), 2);
  sendDisplay(String(targetSpeed), 3);

  sendDisplay(String(accelConvertedValue255), 9);
  sendDisplay(String(accelValue1023), 7);
  sendDisplay(String(autoBrakeValue100), 11);

  if (isBrakePushed) {
    sendDisplay("!BR!", 13);
  } else {
    sendDisplay("", 13);
  }
}

// Util
int boxingInt(int min, int max, int value) {
  if (value <= min) return min;
  else if (value >= max) return max;
  else return value;
}