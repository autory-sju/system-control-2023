#include "mas001.h"
#include "blc200.h"

/*****************************************************************************/
// User Configuration
const int GEAR_RATIO = 250; // Caution!, max -> 192, Reduction ratio is (0.1*GEAR_RATIO)
// max -> 192
const int DEVICE_ID = 0;
int rated_speed = 0;
/*****************************************************************************/

MAS001 myShield;
BLC200 myDevice(9600, 100); // Baudrate = 9600, Serial timeout = 100ms
long data = 0;
uint16_t pos_input; // uint16_t -> 0 ~ 65535

void setup() {
  pos_input=0;
  Serial.begin(115200);

  // [[ Read rated speed from driver ]]
  Serial.print("Read rated speed [RPM] : ");
  if(myDevice.get_Feedback(DEVICE_ID, 0xA6)){
    rated_speed = (uint16_t)myDevice.blcData[1] << 8 | (uint16_t)myDevice.blcData[2];
    Serial.println(rated_speed);
  }else{
    Serial.println("Fail..");
    while(1);
  }

  myDevice.set_ReductionRatio(DEVICE_ID, GEAR_RATIO);

  delay(5000);
  Serial.println("Wait done");
  myDevice.set_PositionWithSpeed(DEVICE_ID, 1,65473, rated_speed * 10);
}

volatile char tmp;
String txtt="";
bool isFirst = true;

void loop() {

  //시리얼로 읽어오는 부분

  //시리얼로 읽은 숫자를 data로 저장 
  
  int spd = 0;
  
  if (Serial.available() > 0){
   data = (int)Serial.readStringUntil('\n').toInt();
  data = data*(65473)/100;
  Serial.print("input data: ");
  Serial.println(data);
  myDevice.set_PositionWithSpeed(DEVICE_ID, 0, data, rated_speed * 10);
  data = 0;

  }


  delay(10);
}