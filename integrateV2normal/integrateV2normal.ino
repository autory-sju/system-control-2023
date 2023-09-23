#include "mas001.h"
#include "blc200.h"
#include <MsTimer2.h>

// analong pin
const int accel_input_pin = 10; //A0

// digital pin
const int speed_input_pin = 20;
const int dcm_output_pin = 47;
const int forward_input_pin = 36;
const int forward_relay_output_pin = 50;
const int auto_pin = 53;

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
float currentSpeed = 0;
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
int estep_read;
int estep;
 
BLC200 linearm(9600, 100);
MAS001 myShield;


void setup() {
  Serial.begin(115200);
  pinMode(auto_pin, INPUT_PULLUP);
  // pinMode(speed_input_pin, INPUT);
  pinMode(dcm_output_pin, OUTPUT);   
  pinMode(forward_input_pin, INPUT_PULLUP);
  pinMode(forward_relay_output_pin, OUTPUT);

  // attachInterrupt(digitalPinToInterrupt(speed_input_pin), encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(speed_input_pin), encoder, HIGH);

  if(linearm.get_Feedback(LENEAR_DEVICE_ID, 0xA6)){
    linearSpeed = (uint16_t)linearm.blcData[1] << 8 | (uint16_t)linearm.blcData[2];
  }/* else{
    while(true){
      Serial.println("brake error");
      
    }
  }*/
  
  Serial.println("plz wait 5secs");
  linearm.set_ReductionRatio(LENEAR_DEVICE_ID, LINEAR_GEAR_RATIO);
  delay(5000);
  Serial.println("initialized");
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position

  MsTimer2::set(200, renewCurrentSpeedInterrupt);
  MsTimer2::start();
}

void loop() {
  autoSwitch();
  forwardSwitch();


  if(isAuto){
    
    if (Serial.available() > 0) {   // 시리얼 버퍼에 데이터가 있는지 확인
      targetSpeed =(int) Serial.parseInt();
      Serial.read();

      speedDis = targetSpeed - currentSpeed;
      speedDis = boxingInt(-2, 20, speedDis);

      if(speedDis >= -2){
        autoAcceleration(map(speedDis, -2, 20, 30, 100)); //0%(=30)<= speedDis <= 100%(100)
      } else if(-5 > speedDis){
        autoBrake(-speedDis * 5); //0%(=25)<= speedDis <= 100%(50)
      }
    }
  }
  else{
    manualAcceleration();
  }
  
}

// Tractive Control
void autoBrake(int brakePressPercent){
  // dcm stop
  analogWrite(dcm_output_pin, 0);

  brakePressPercent = boxingInt(0, 100, brakePressPercent);
  
  if(brakePressPercent >= 80) brakePressPercent = 100;
  else if(brakePressPercent >= 40) brakePressPercent = 50;
  else if(brakePressPercent >= 10) brakePressPercent = 10;
  else brakePressPercent = 0;
  Serial.print(brakePressPercent);
  Serial.println("% brake");

  int convertedBrakeValue = map(brakePressPercent, 0, 100, 0, 20);
  int brakeInternalValue = convertedBrakeValue * (65473) / 100;

  accelConvertedValue255=0;
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 0, brakeInternalValue, linearSpeed * 10);
}

void autoAcceleration(int accelPressPercent){
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position

  accelPressPercent = boxingInt(0, 100, accelPressPercent);

  Serial.print(accelPressPercent);
  Serial.println("% accel AUTO");
  accelConvertedValue255 += map(accelPressPercent, 0, 100, 0, 100);
  analogWrite(dcm_output_pin, accelPressPercent);
}

void manualAcceleration(){
  // brake realease
  linearm.set_PositionWithSpeed(LENEAR_DEVICE_ID, 1, 65473, linearSpeed * 10); // go initial position

  accelValue1023 = analogRead(accel_input_pin);   // read analog accel input
  accelConvertedValue255 = map(accelValue1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);
  Serial.print(map(accelConvertedValue255,0,255,0,100));
  Serial.println("% accel MANUAL");
  analogWrite(dcm_output_pin, accelConvertedValue255);
}

// MODE SETTING
void autoSwitch(){
  bool prev = isAuto;
  if(digitalRead(auto_pin) == 1) isAuto = true;
  else {
    isAuto = false;
  }
  if(prev!=isAuto){
    analogWrite(dcm_output_pin, accelConvertedValue255);
  }
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


// Speed Encoding
void encoder(){
  estep++;
}
void renewCurrentSpeedInterrupt(){
  estep_read = estep;
  estep = 0;
  // if(estep_read >= 6) estep_read = 6;
  currentSpeed =  PI * WHEEL_DIAMETER * 0.72 * estep_read; // m/s for every 0.2sec
  Serial.print("CurrentSpeed/step: ");
  Serial.print(currentSpeed);
  Serial.print("km/h / ");
  Serial.println(estep_read);
}

int boxingInt(int min, int max, int value){
  if(value<=min) return min;
  else if(value>=max) return max;
  else return value;
}
