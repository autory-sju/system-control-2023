#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "mas001.h"
#include "blc200.h"
#include <MsTimer2.h>
#include <string.h>
#include <SoftwareSerial.h>


// analong pin
const int accel_input_pin = 10;  //A0

// digital pin
const int speed_input_pin = 20;
const int dcm_output_pin = 44;
const int forward_R_pin = 32;
// const int forward_N_pin = 38;
const int forward_D_pin = 33;
const int forward_relay_output_pin = 50;
const int auto_ON_pin = 35;
const int auto_EM_pin = 34;
//const int auto_OFF_pin = 42;
const int foot_pin = 43;
const int display_rx = 31;
const int display_tx = 30;

// constant
const int MAX_PEDAL_INPUT_1023 = 880;  // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 180;       // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;
const int LINEAR_GEAR_RATIO = 250;  // max would be 192
const int LENEAR_DEVICE_ID = 0;
const int LENEAR_LIMIT_LENGTH = 20;

// variable
char buf[30];
// bool isForward = true;
int directionMode = 0;
bool isAuto = true;
int autoMode = 0;
int targetSpeed = 0;
float currentSpeed = 0;
int speedDis = 0;
int dcmOutputValue = 0;
int linearSpeed = 30000;
int accelValue1023 = 0;
int accelConvertedValue255 = 0;
unsigned long curTime = 0;
unsigned long prevEncodedTime = 0;
unsigned long periodTime = 0;
int currentIRState = 0;
int prevIRState = 0;
float RPM = 0;
int estep_read;
int estep;
float err_P = 0, err_I = 0, err_D = 0, err_B = 0;
float Pv = 0.02, Iv = 0.02;  // 반응 속도
float Dv = 2;                // 급격한 변화 방지 (오버슈팅 방지)

// Ros
void messageCb(const std_msgs::Int32& msg);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> sub("tractive_control", &messageCb);
std_msgs::String str_msg;
ros::Publisher pub("arduino_out", &str_msg);
void messageCb(const std_msgs::Int32& msg) {
  targetSpeed = msg.data;
}

BLC200 linearm(9600, 100);
MAS001 myShield;
SoftwareSerial HMISerial(display_rx, display_tx);  // RX, TX

void setup() {


  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  pinMode(auto_ON_pin, INPUT_PULLUP);
  pinMode(auto_EM_pin, INPUT_PULLUP);
//  pinMode(auto_OFF_pin, INPUT_PULLUP);
  pinMode(foot_pin, INPUT_PULLUP);
  pinMode(forward_R_pin, INPUT_PULLUP);
//  pinMode(forward_N_pin, INPUT_PULLUP);
  pinMode(forward_D_pin, INPUT_PULLUP);

  pinMode(dcm_output_pin, OUTPUT);
  pinMode(forward_relay_output_pin, OUTPUT);

  HMISerial.begin(9600);
    while (!Serial) {}  // 시리얼 포트가 연결될 대까지 기다림

  Serial.println("시리얼 포트 연결됨");


  // attachInterrupt(digitalPinToInterrupt(speed_input_pin), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(speed_input_pin), encoder, HIGH);

  if (linearm.get_Feedback(LENEAR_DEVICE_ID, 0xA6)) {
    linearSpeed = (uint16_t)linearm.blcData[1] << 8 | (uint16_t)linearm.blcData[2];
  } else {
    while (true) {
      // Serial.println("brake error");
    }
  }

  Serial.println("plz wait 5secs");
  linearm.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  delay(3000);
  Serial.println("initialized");
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);  // go initial position

  // MsTimer2::set(200, renewCurrentSpeedInterrupt);
  // MsTimer2::start();
}

void loop() {
  nh.spinOnce();
  autoModeSwitch();
  driveDirectionSwitch();

    sendDisplay(String(millis()), 2);

  // if (autoMode == 1) {
  //   targetSpeed = (int)Serial.readStringUntil('\n').toInt();
  //   sendDisplay(String(targetSpeed), 3);
  //   speedDis = targetSpeed - currentSpeed;
  //   speedDis = boxingInt(-2, 20, speedDis);

  //   if (speedDis >= -2) {
  //     autoAcceleration(map(speedDis, -2, 40, 20, 65));
  //     //0%(=30)<= speedDis <= 100%(100) MAX65%
  //   } else if (-5 > speedDis) {
  //     linearControl(50);
  //     //0%(=25)<= speedDis <= 100%(50)
  //   }
  // } else if (autoMode = 2) {
  //   linearControl(100);
  // } else if (autoMode = 3) {
  //   manualAcceleration();


  // }
}


// Tractive Control
// void autoBrake(int brakePressPercent) {
//   // dcm stop
//   analogWrite(dcm_output_pin, 0);

//   brakePressPercent = boxingInt(0, 100, brakePressPercent);

//   if (brakePressPercent >= 80) brakePressPercent = 100;
//   else if (brakePressPercent >= 40) brakePressPercent = 50;
//   else if (brakePressPercent >= 10) brakePressPercent = 10;
//   else brakePressPercent = 0;
//   Serial.print(brakePressPercent);
//   Serial.println("% brake");

//   int convertedBrakeValue = map(brakePressPercent, 0, 100, 0, 20);
//   int brakeInternalValue = convertedBrakeValue * (65473) / 100;

//   accelConvertedValue255 = 0;
//   linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, brakeInternalValue, linearSpeed * 10);
// }

void autoAcceleration(int accelPressPercent) {
  // // brake realease
  // linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);  // go initial position

  //accelPressPercent = boxingInt(0, 100, accelPressPercent);
  err_P = speedDis;
  err_I += err_P;
  err_D = err_B - err_P;
  err_B = err_P;
  accelPressPercent = ((err_P * Pv) + (err_I * Iv) + (err_D * Dv));
  accelPressPercent = boxingInt(0, 100, accelPressPercent);
  dcmControl(accelPressPercent);

  Serial.print(accelPressPercent);
  Serial.println("% accel AUTO");
}

void manualAcceleration() {
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);  // go initial position

  accelValue1023 = analogRead(accel_input_pin);  // read analog accel input
  dcmControl(map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 100));
  // accelConvertedValue255 = map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);
  // Serial.print(map(accelConvertedValue255, 0, 255, 0, 100));
  // Serial.println("% accel MANUAL");
  // analogWrite(dcm_output_pin, accelConvertedValue255);
}

// MODE SETTING
void autoModeSwitch() {

  //ON40 EM41 OFF42
  int prevMode = autoMode;
  if (digitalRead(auto_ON_pin) == 0 && digitalRead(auto_EM_pin) == 1) {
    isAuto = true;
    autoMode = 1;
  } else if (digitalRead(auto_ON_pin) == 1 && digitalRead(auto_EM_pin) == 0) {
    isAuto = true;
    autoMode = 2;
  }// else if (digitalRead(auto_OFF_pin) == 1) {
  //  isAuto = false;
  //  autoMode = 3;
  //}
   else {
    isAuto = false;
    autoMode = 3;
  }

  if (prevMode != autoMode) {
    analogWrite(dcm_output_pin, 0);
    if (autoMode == 1) sendDisplay("AS-ON", 14);
    else if (autoMode == 2) sendDisplay("AS-EMER", 14);
    else if (autoMode == 3) sendDisplay("AS-OFF", 14);
    else sendDisplay("AS-OFF", 14);
  }
}
void driveDirectionSwitch() {
  int prevMode = directionMode;

  if (digitalRead(forward_R_pin) == 0 &&digitalRead(forward_D_pin) == 1) {
    directionMode = 1;
    digitalWrite(forward_relay_output_pin, HIGH);
  }  //else if (digitalRead(forward_N_pin) == 1) {
     //directionMode = 2;
     //digitalWrite(forward_relay_output_pin, LOW);
     //}
  else if (digitalRead(forward_R_pin) == 1 &&digitalRead(forward_D_pin) == 0) {
    directionMode = 3;
  } else {
    directionMode = 2;
    digitalWrite(forward_relay_output_pin, LOW);
  }
  if (prevMode != directionMode) {
    analogWrite(dcm_output_pin, 0);
    if (directionMode == 1) sendDisplay("R", 5);
    else if (directionMode == 2) sendDisplay("N", 5);
    else if (directionMode == 3) sendDisplay("D", 5);
  }

  // if (digitalRead(forward_input_pin) == 1) {
  //   isForward = true;
  //   digitalWrite(forward_relay_output_pin, HIGH);
  // } else {
  //   isForward = false;
  //   digitalWrite(forward_relay_output_pin, LOW);
  // }
}

// Speed Encoding
void encoder() {
  if (prevEncodedTime - millis() > 30) {
    estep++;
    prevEncodedTime = millis();
  }
}
void renewCurrentSpeedInterrupt() {

  estep_read = estep;
  estep = 0;
  // if(estep_read >= 6) estep_read = 6;
  currentSpeed = PI * WHEEL_DIAMETER * 0.72 * estep_read;  // m/s for every 0.2sec
  // Serial.print("CurrentSpeed/step: ");
  // Serial.print(currentSpeed);
  // Serial.print("km/h / ");
  // Serial.println(estep_read);
  dtostrf(currentSpeed, 4, 2, buf);
  str_msg.data = buf;
  pub.publish(&str_msg);
  sendDisplay(String(currentSpeed), 2);
}



// MOTOR CONTROL IN 0~100%
void dcmControl(int percent) {
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);

  percent = boxingInt(0, 100, percent);
  accelConvertedValue255 = map(percent, 0, 100, 0, 255);
  analogWrite(dcm_output_pin, accelConvertedValue255);
  sendDisplay(String(percent), 9);
}

void linearControl(int percent) {
  // dcm stop
  analogWrite(dcm_output_pin, 0);

  percent = boxingInt(0, 100, percent);
  int pressLength = 0;
  if (percent >= 80) pressLength = 20;  // in mm
  else if (percent >= 40) pressLength = 10;
  else if (percent >= 10) pressLength = 5;
  else pressLength = 0;

  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, pressLength * (65473) / 100, linearSpeed * 10);
  sendDisplay(String(percent), 11);
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
      target = "t7.txt=\"";  // foot
      break;
    case 11:
      target = "t11.txt=\"";  // brake
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
  Serial.print(mode);
  Serial.println(sendData);
  for (int i = 0; i < target.length(); i++) {
    HMISerial.write(target[i]);  // send each char
  }
  HMISerial.write(0xff);
  HMISerial.write(0xff);
  HMISerial.write(0xff);  // end signal byte
}

// Util
int boxingInt(int min, int max, int value) {
  if (value <= min) return min;
  else if (value >= max) return max;
  else return value;
}