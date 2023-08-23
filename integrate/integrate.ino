#include "mas001.h"
#include "blc200.h"

// analong pin
const int accel_input_pin = 10; //A0

// digital pin
const int speed_input_pin = 49;
const int dcm_output_pin = 43;
const int forward_input_pin = 40;
const int forward_relay_output_pin = 38;

// constant
const int MAX_PEDAL_INPUT_1023 = 880;   // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 190;   // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;  // 0.51
const int LINEAR_GEAR_RATIO = 250; // max would be 192
const int LENEAR_DEVICE_ID = 0;
const int LENEAR_LIMIT_LENGTH = 50;  

// variable
char buf[80];
bool isForward = true;
int targetSpeed = 0;
int currentSpeed = 0;
int speedDis = 0;
int dcmOutputValue = 0;
int linearSpeed = 30000;
int accelValue1023 = 0;
int accelConvertedValue255 = 0;
unsigned long curTime = 0;
unsigned long prevTime = 0;
unsigned long periodTime = 0;
int currentIRState = 0;
int prevIRState = 0;
float RPM = 0;


MAS001 myShield;
BLC200 myDevice(9600, 100); // Baudrate = 9600, Serial timeout = 100ms


void setup() {
  Serial.begin(115200);
  pinMode(speed_input_pin, INPUT);
  pinMode(dcm_output_pin, OUTPUT);   // 9번 핀을 출력으로 설정 (모터를 제어하는 출력 핀으로 사용)
  pinMode(forward_input_pin, OUTPUT);
  pinMode(forward_relay_output_pin, INPUT);

  if(myDevice.get_Feedback(LENEAR_DEVICE_ID,  0xA6)){
    linearSpeed = (uint16_t)myDevice.blcData[1] << 8 | (uint16_t)myDevice.blcData[2];
    Serial.print("linearSpeed: ");
    Serial.println(linearSpeed);
  } else{
    Serial.println("Fail..");

    while(!myDevice.get_Feedback(LENEAR_DEVICE_ID,  0xA6)); //error led on
  }
  
  myDevice.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  delay(5000);
  Serial.println("READY");
  myDevice.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position
}

void loop() {
  renewCurrentspeed();
  
  if (Serial.available() > 0) {   // 시리얼 버퍼에 데이터가 있는지 확인
    targetSpeed = Serial.parseInt();
    Serial.print("targetSpeed: ");
    Serial.println(targetSpeed);

    speedDis = targetSpeed - currentSpeed;

    if(speedDis >= -2){
      Serial.print("accel: ");
      autoAcceleration(map(speedDis, -2, 10, 30, 100));
    } else if(-5 > speedDis && targetSpeed!= 0){
      Serial.print("brake: ");
      autoBrake(-speedDis * 5);
    }

  }
}

void autoBrake(int brakePress){
  if(brakePress > LENEAR_LIMIT_LENGTH) brakePress = LENEAR_LIMIT_LENGTH;
  Serial.println(brakePress);
  brakePress = brakePress * (65473) / 100;
  myDevice.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, brakePress, linearSpeed * 10);
}

void autoAcceleration(int accelPress){
  accelConvertedValue255 = map(accelPress, 0, 100, 0, 255);
  Serial.println(accelConvertedValue255);
  analogWrite(dcm_output_pin, accelConvertedValue255);

}

void manualAcceleration(){
  accelValue1023 = analogRead(accel_input_pin);   // 아날로그 입력핀으로부터 페달 입력 값을 읽어옴
  accelConvertedValue255 = map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);
  analogWrite(dcm_output_pin, accelConvertedValue255);
}


void renewCurrentspeed(){
  currentIRState = digitalRead(speed_input_pin);
  if(currentIRState == 1 && currentIRState != prevIRState && (millis() - prevTime) > 90){
    curTime = millis();
    periodTime = curTime - prevTime;
    RPM = 60000 / periodTime;
    prevTime = curTime;
    currentSpeed = WHEEL_DIAMETER * PI * RPM * 60 / 1000;
    Serial.print("currentSpeed: ");
  Serial.println(currentSpeed);
  }
  prevIRState = currentIRState;
}


