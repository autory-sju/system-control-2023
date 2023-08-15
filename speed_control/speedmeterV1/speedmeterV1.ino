#include <Arduino.h>
#include <TimerOne.h>

#define T    2 

const float Diameter = 0.51; float lnTime = 0;
long endTime=0; long startTime; int lin_vel=0;
int RPM = 0; long lastDebounceTime = 0; int lastInputState = LOW;        

void setup() {
  Serial.begin(9600);
  pinMode(T, INPUT);
} 
 
void loop() {
  int currentSwitchState = digitalRead(T);  // get state
  if (currentSwitchState != lastInputState && currentSwitchState==1 ) { // 한바퀴 돌았을 때
    lastDebounceTime = millis();
    calculateRPM(); // rpm 계산
    timerIsr();  // 속도 계산 및 출력
    delay(80); // 딜레이 통한 무분별한 탐지 조정  현 최대속도 72km/h
  }
  lastInputState = currentSwitchState;
}

void calculateRPM() {
  startTime = lastDebounceTime;
  lnTime = startTime - endTime;  // 한바퀴 도는데 걸린 시간 
  RPM = 60000 / lnTime; // 1초 == 1000 이라서 60000에 나눈다.
  endTime = startTime;
}

void timerIsr() {
  // RPM 계산
  lin_vel = (Diameter  * PI ) * RPM * 60 / 1000 ;// 속도 계산 
  Serial.print("Linear Speed: ");
  Serial.println(lin_vel); // 출력
}