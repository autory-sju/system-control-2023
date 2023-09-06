#include "mas001.h"
#include "blc200.h"

// analong pin
const int accel_input_pin = 10; //A0

// digital pin
const int speed_input_pin = 20;
const int dcm_output_pin = 44;
const int forward_input_pin = 38;
const int forward_relay_output_pin = 50;
const int auto_pin = 40;

// constant
const int MAX_PEDAL_INPUT_1023 = 880;   // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 180;   // 최소 페달 입력값 (0-1023 범위) 설정
const float WHEEL_DIAMETER = 0.51;
const int LINEAR_GEAR_RATIO = 250; // max would be 192
const int LENEAR_DEVICE_ID = 0;
const int LENEAR_LIMIT_LENGTH = 20;  

// variable
char buf[80];
bool isForward = true;
bool isAuto = true;
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
 
BLC200 linearm(9600, 100);
MAS001 myShield;


void setup() {
  Serial.begin(115200);
  pinMode(auto_pin, INPUT_PULLUP);
  pinMode(speed_input_pin, INPUT);
  pinMode(dcm_output_pin, OUTPUT);   
  pinMode(forward_input_pin, INPUT_PULLUP);
  pinMode(forward_relay_output_pin, OUTPUT);

  if(linearm.get_Feedback(LENEAR_DEVICE_ID, 0xA6)){
    linearSpeed = (uint16_t)linearm.blcData[1] << 8 | (uint16_t)linearm.blcData[2];
  } else{
    while(true){
      Serial.println("brake error");
      
    }
    // while(1); error led on
  }
  
  Serial.println("plz wait 5secs");
  linearm.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  delay(5000);
  Serial.println("initialized");
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position
}

void loop() {
  renewCurrentspeed();
  autoSwitch();
  forwardSwitch();

  if(isAuto){
    
    if (Serial.available() > 0) {   // 시리얼 버퍼에 데이터가 있는지 확인
      targetSpeed = Serial.parseInt();
      Serial.read();

      speedDis = targetSpeed - currentSpeed;

      if(speedDis >= -2){
        autoAcceleration(map(speedDis, -2, 10, 30, 100)); //0%(=30)<= speedDis <= 100%(100)
      } else if(-5 > speedDis){
        autoBrake(-speedDis * 5); //0%(=25)<= speedDis <= 100%(50)
      }
    }
  }
  else{
    manualAcceleration();
  }
  
}

void autoBrake(int brakePress){
  if(brakePress > LENEAR_LIMIT_LENGTH) brakePress = LENEAR_LIMIT_LENGTH;
  Serial.print(brakePress);
  Serial.println("% brake");
  brakePress = brakePress * (65473) / 100;

  // dcm stop
  analogWrite(dcm_output_pin, 0);

  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, brakePress, linearSpeed * 10);
}

void autoAcceleration(int accelPress){
  dataPrint();
  Serial.print(accelPress);
  Serial.println("% accel AUTO");
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position
  accelConvertedValue255 += map(accelPress, 0, 100, 0, 255);
  analogWrite(dcm_output_pin, accelConvertedValue255);
}

void manualAcceleration(){
  dataPrint();
  accelValue1023 = analogRead(accel_input_pin);   // 아날로그 입력핀으로부터 페달 입력 값을 읽어옴
  accelConvertedValue255 = map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);
  if(accelConvertedValue255<=0) accelConvertedValue255 = 0;
  if(accelConvertedValue255>=255) accelConvertedValue255 = 255;

  // Serial.print(map(accelConvertedValue255,0,255,0,100));
  // Serial.println("% accel MANUAL");
  // Serial.print("Origin:");
  // Serial.println(accelValue1023);

  analogWrite(dcm_output_pin, accelConvertedValue255);
}


void autoSwitch(){
  if(digitalRead(auto_pin) == 1) isAuto = true;
  else isAuto = false;
}

void forwardSwitch(){
  if(digitalRead(forward_input_pin) == 1) {
    digitalWrite(forward_relay_output_pin, HIGH); 
    // Serial.println("HIGH");
  } else {
    digitalWrite(forward_relay_output_pin, LOW);
    // Serial.println("LOW");
  }
}

void renewCurrentspeed(){
  currentIRState = digitalRead(speed_input_pin);
  if(currentIRState == 1 && currentIRState != prevIRState && ( millis() - prevTime) > 90){
    curTime = millis();
    periodTime = curTime - prevTime;
    RPM = 60000 / periodTime;
    prevTime = curTime;
    currentSpeed = WHEEL_DIAMETER * PI * RPM * 60 / 1000;
    Serial.print("currentspd: ");
    Serial.println(currentSpeed);
  }
  prevIRState = currentIRState;
}

void dataPrint(){
  Serial.print(isAuto ? "auto " : "manual ");
  Serial.print("CurSpeed: ");
  Serial.println(currentSpeed);
  
}


