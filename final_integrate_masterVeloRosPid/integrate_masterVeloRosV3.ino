#include "mas001.h"
#include "blc200.h"

// analong pin
const int accel_input_pin = 15;

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

// const int display_rx_pin = 31;
// const int display_tx_pin = 30;

// const int steer_rx_pin = 33;
// const int steer_tx_pin = 32;

unsigned long lastSpdUpdTime = 0;


// constant
const int MAX_PEDAL_INPUT_1023 = 700;  // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 200;       // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;
const int LINEAR_GEAR_RATIO = 250;  // max would be 192
const int LENEAR_DEVICE_ID = 0;
const float LENEAR_LIMIT_LENGTH = 14;

// variable
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
int i = 0;

char serial1Char;
String serial1Str = "";
bool isStrRead = false;
int speedDisInt = 0;

float err_P = 0, err_I = 0, err_D = 0, err_B = 0;
float Pv = 0.008, Iv = 0.008;  // 반응 속도
float Dv = 1;                  // 급격한 변화 방지 (오버슈팅 방지)
long pressLengthFinal = 0;
float pressLength = 0;

BLC200 linearm(9600, 100);
MAS001 myShield;

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
  // linearControl(0);
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);
  
  delay(3000);
  Serial1.flush();
}


void loop() {
  autoModeSwitch();
  driveDirectionSwitch();
  vesSwitch();
  brakeSwitch();
  showingDisplay();

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
      } else {
        speedDis = 0;
      }
      break;
    } else {
      if (isStrRead) serial1Str += serial1Char;
    }
    // Serial.println(serial1Char);
  }

  if (autoMode == 1) {  //  AS - ON
                        // current speed -> ROS velocity
                        // target speed -> ROS
    if (prevAutoMode != autoMode) {
      err_P = 0;
      err_I = 0;
      err_D = 0;
      err_B = 0;
    }

    speedDis = boxingFloat(-500.0, 1000.0, speedDis);

    err_P = speedDis;
    err_I += err_P;
    err_D = err_B - err_P;
    err_B = err_P;


    // 종방향
    if (speedDis >= 0) {
      autoAcceleration();  //0%(=30)<= speedDis <= 100%(100) MAX65%
    } else if (-100 > speedDis) {
      linearControl(50);  //0%(=25)<= speedDis <= 100%(50)
    }
  } else if (autoMode == 2) {  // AS - EM
    linearControl(100);
  } else if (autoMode == 3) {  // AS - OFF
    manualAcceleration();
  }
  prevAutoMode = autoMode;
}

void autoAcceleration() {

  accelAutoPressPercent = ((err_P * Pv) + (err_I * Iv) + (err_D * Dv));
  accelAutoPressPercent = boxingInt(0, 60, accelAutoPressPercent);
  sendDisplay("AUTO " + String(accelAutoPressPercent), 15);
  if (directionMode != 2) {
    dcmControl(accelAutoPressPercent);
  }
}

void manualAcceleration() {
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);

  if (lastSpdUpdTime - millis() >= 100) {
    accelValue1023 = analogRead(accel_input_pin);  // read analog accel input
    lastSpdUpdTime = millis();
  }


  if (directionMode != 2) {
    accelManualPressPercent = map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 100);
    accelManualPressPercent = boxingInt(0, 100, accelManualPressPercent);
    dcmControl(accelManualPressPercent);

    // for(i=0;i<=accelManualPressPercent;i++){
    //   dcmControl(accelManualPressPercent);
    // }
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
  if (directionMode == 2 || isBrakePushed == 1) percent = 0;
  percent = boxingInt(0, 100, percent);
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10);

  if (isBrakePushed) {
    analogWrite(dcm_output_pin, 0);
  } else {
    percent = boxingInt(0, 100, percent);
    accelConvertedValue255 = map(percent, 0, 100, 0, 200);  // max 150 out of 255
    analogWrite(dcm_output_pin, accelConvertedValue255);
  }
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
  pressLengthFinal = pressLength * (65473) / 100;

  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, pressLengthFinal, linearSpeed * 10);
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
    Serial2.write(target[i]);  // send each char
  }
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);  // end signal byte
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
float boxingFloat(float min, float max, float value) {
  if (value <= min) return min;
  else if (value >= max) return max;
  else return value;
}