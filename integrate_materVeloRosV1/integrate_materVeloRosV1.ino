// #include <ArduinoHardware.h>
// #include <ros.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Float64.h>
// //#include <geometry_msgs/TwistWithCovarianceStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/String.h>
#include "mas001.h"
#include "blc200.h"
//#include <SoftwareSerial.h>

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
const int MAX_PEDAL_INPUT_1023 = 810;  // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 180;       // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;
const int LINEAR_GEAR_RATIO = 250;  // max would be 192
const int LENEAR_DEVICE_ID = 0;
const int LENEAR_LIMIT_LENGTH = 15;

// variable
char buf[30];
int directionMode = 0;
bool isAuto = true;
int autoMode = 0;
int prevAutoMode = 0;
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
int accelManualPressPercent = 0;

char serial1Char;
String serial1Str = "";
bool isStrRead = false;
int speedDisInt = 0;

float err_P = 0, err_I = 0, err_D = 0, err_B = 0;
float Pv = 0.002, Iv = 0.002;  // 반응 속도
float Dv = 2;                  // 급격한 변화 방지 (오버슈팅 방지)
long pressLength = 0;

BLC200 linearm(9600, 100);
MAS001 myShield;
//SoftwareSerial HMISerial(display_rx_pin, display_tx_pin);  // RX, TX


void setup() {
  pinMode(auto_ON_pin, INPUT_PULLUP);
  pinMode(auto_OFF_pin, INPUT_PULLUP);
  pinMode(ves_switch_pin, INPUT_PULLUP);
  pinMode(forward_R_switch_pin, INPUT_PULLUP);
  pinMode(forward_D_switch_pin, INPUT_PULLUP);
  pinMode(brake_switch_pin, INPUT_PULLUP);


  pinMode(dcm_output_pin, OUTPUT);
  pinMode(forward_relay_output_pin, OUTPUT);
  pinMode(ves_output_pin, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(9600);
  while (!Serial) {
    Serial.println("Serial out");
  }  // 시리얼 포트가 연결될 대까지 기다림
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
  // str_msg.data = "publisher initiallized";
  // pub.publish( &str_msg );
  linearm.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  Serial.println("initialized");
  linearControl(0);
  delay(3000);
  Serial1.flush();
}


void loop() {
  autoModeSwitch();
  driveDirectionSwitch();
  vesSwitch();
  brakeSwitch();
  showingDisplay();

  // if(Serial1.available() == SERIAL_RX_BUFFER_SIZE -1) {
  //   Serial.read();Serial.println("Del");}
  Serial.println("LOOP" + String(Serial1.available()));
  while (Serial1.available() >= 4) {

    serial1Char = Serial1.read();
    if (serial1Char == 's') {
      serial1Str = "";
      isStrRead = true;
    } else if (serial1Char == 'f' && isStrRead) {
      isStrRead = false;
      speedDisInt = (int)serial1Str.toInt();
      if (abs(speedDisInt) >= 10) {
        speedDis = ((float)speedDisInt);
        sendDisplay(String(speedDis), 2);
        sendDisplay(String(speedDisInt), 3);
        Serial.print("GOT:");
        Serial.print(speedDis + 0.01);
        Serial.print("  GOTINT:");
        Seria