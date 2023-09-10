#include <SoftwareSerial.h>


SoftwareSerial HMISerial(2, 3);  // RX, TX



void setup() {
  Serial.begin(9600);
  HMISerial.begin(9600);
  while (!Serial) {}  // 시리얼 포트가 연결될 대까지 기다림
  Serial.println("시리얼 포트 연결됨");
}



void loop() {
  sendString(String(millis()), 2);
  if (Serial.available() > 0) {

  String outData = Serial.readStringUntil('\n');  // 송신 문자
  sendString(outData, 3);
  Serial.println(outData);

  delay(100);
  }
}


void sendString(String sendData, int mode) {
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
    // case 7:
    //   target = "t7.txt=\"";  // target
    //   break;
    // case 8:
    //   target = "t8.txt=\"";  // target
    //   break;
    // case 9:
    //   target = "t9.txt=\"";  // target
    //   break;
    default:
      return;
  }
  target.concat(sendData);
  target.concat("\"");
  for (int i = 0; i < target.length(); i++) {
    HMISerial.write(target[i]);  // 1 문자씩 보냄
  }
  HMISerial.write(0xff);
  HMISerial.write(0xff);
  HMISerial.write(0xff);  // 종료 바이트 3개 보냄
}
