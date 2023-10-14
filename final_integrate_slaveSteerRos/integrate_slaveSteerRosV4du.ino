const int steer_step_pin = 8;
const int steer_dir_pin = 9;
const int steer_en_pin = 10;
const int switch_pin = 2;

const int MAX_STEER_L_STEP = 8000;
const int MAX_STEER_R_STEP = -8000;

const int STEP_DELAY = 150;  //현석


float mtSpeed = 0;               // 초기 속도 설정 (0은 정지 상태)
int stepDirection = 0;           // 초기 stepDirection: 0은 정지, 1은 시계 반대 방향, -1은 시계 방향
int REDUCER_GEAR_RATIO = 20;     // 감속기 기어비
float GEARBOX_RATIO = 1.5;       // 조향하우징 기어비
long totalSteps = 0;             // 총 이동 스텝 수
bool isStepMotorStopped = true;  // 모터가 현재 정지한지 여부
int prevState = 0;
float steer_velocity = 0;

unsigned long t = 0;


char serialChar;
String serialStr = "";
bool isStrRead = false;
bool isValueCompl = false;
int steerVelocityInt = 0;

bool isWork = false;


void setup() {
  pinMode(steer_step_pin, OUTPUT);
  pinMode(steer_dir_pin, OUTPUT);
  pinMode(steer_en_pin, OUTPUT);

  pinMode(switch_pin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(switch_pin), makeStop, LOW);
  // attachInterrupt(digitalPinToInterrupt(switch_pin), makeWork, RISING);


  //digitalWrite(steer_en_pin, LOW);  // 초기에 모터 비활성화


  Serial.begin(38400);     // 시리얼 통신 속도 설정
  Serial.setTimeout(100);  // 입력 파싱을 위한 타임아웃 설정
}

void loop() {
  readEnablePin();
  // Serial.println("enable stts " + String(isWork));

  // Serial.println("Lwwoop " + String(Serial.available()));
  // Serial.println("VAL " + String(steer_velocity));
  while(Serial.available() > 10) Serial.read();
  //Serial.println("buf" + String(Serial.available()));
  while (Serial.available() > 0) {
    serialChar = Serial.read();
    if (serialChar == 's') {
      serialStr = "";
      isStrRead = true;
    } else if (serialChar == 'f' && isStrRead) {
      isStrRead = false;
      steerVelocityInt = (int)serialStr.toInt();
      isValueCompl = true;
      steer_velocity = ((float)steerVelocityInt);

      // if (abs(steerVelocityInt) >= 1) {
      //   steer_velocity = ((float)steerVelocityInt);
      // } else {
      //   steer_velocity = 0;
      // }
      break;
    } else {
      if (isStrRead) serialStr += serialChar;
    }
  }

  if (isValueCompl && isWork) {
    // 입력의 부호에 따라 stepDirection 결정
    if (steer_velocity < 0) {
      stepDirection = -1;  // 음수 입력일 때 시계 방향
      // digitalWrite(steer_en_pin, LOW);
      digitalWrite(steer_dir_pin, LOW);
    } else if (steer_velocity > 0) {
      stepDirection = 1;  // 양수 입력일 때 시계 반대 방향
      // digitalWrite(steer_en_pin, LOW);
      digitalWrite(steer_dir_pin, HIGH);
    } else {
      stepDirection = 0;  // 0 입력일 때 정지
      digitalWrite(steer_step_pin, HIGH); // HIGH면 저항이 있다.
      digitalWrite(steer_en_pin, LOW);  

    }
    Serial.println("s vel: " + String(steer_velocity / 100));

    steer_velocity = abs(steer_velocity) /100;

    if (stepDirection != 0) {
      t = 0;
      // 속도를 기반으로 지연 시간 계산
      while (t < 200000 * steer_velocity) {
        // Step 신호 ON
        digitalWrite(steer_step_pin, HIGH);
        delayMicroseconds(STEP_DELAY);
        t += STEP_DELAY;
        // Step 신호 OFF
        digitalWrite(steer_step_pin, LOW);
        delayMicroseconds(STEP_DELAY);
        t += STEP_DELAY;
      }
      mtSpeed = 0;
    }
    isValueCompl = false;
  }

  // 0.3sec -> 0.48rad
  // 0.71sec -> 1rad

}

void readEnablePin(){
  if(digitalRead(2) == 1){
      isWork = 1;
      digitalWrite(steer_en_pin, LOW);
  }else{
      isWork = 0;
      digitalWrite(steer_en_pin, HIGH);
  }
}


// void makeStop() { // 사람이 운전하고 싶을때 -> OFF
//   //digitalWrite(steer_step_pin, LOW);
//   digitalWrite(steer_en_pin, HIGH);

//   steer_velocity = 0;
//   isValueCompl = false;
//   Serial.println("OFF");
// }

// void makeWork(){
//     digitalWrite(steer_en_pin, LOW);
//       Serial.println("again");

// }