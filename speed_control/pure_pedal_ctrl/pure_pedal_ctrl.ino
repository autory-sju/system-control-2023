const int MAX_PEDAL_INPUT_1023 = 880;   // 최대 페달 입력값 (0-1023 범위) 설정
const int MIN_PEDAL_INPUT = 170;   // 최소 페달 입력값 (0-1023 범위) 설정
const int throttle_output_pin = 9;
const int pedal_input_pin = 0;
char buf[80];

void setup() {
  Serial.begin(9600);   // 시리얼 통신을 초기화하고 속도를 9600 bps로 설정
  pinMode(throttle_output_pin, OUTPUT);   // 9번 핀을 출력으로 설정 (모터를 제어하는 출력 핀으로 사용)
}

void loop() {
  if (Serial.available() > 0) {   // 시리얼 버퍼에 데이터가 있는지 확인
    int serialInputVal = Serial.parseInt();   // 시리얼 입력 값을 정수로 읽어옴

    // 입력 값을 MIN_PEDAL_INPUT MAX_PEDAL_INPUT_1023 범위로 제한
    if (serialInputVal < MIN_PEDAL_INPUT) {
      serialInputVal = MIN_PEDAL_INPUT;
    } else if (MAX_PEDAL_INPUT_1023 < serialInputVal) {
      serialInputVal = MAX_PEDAL_INPUT_1023;
    }

    Serial.print(serialInputVal);
    Serial.println("/255 -> Input Value");
    // 입력 값을 0-255 범위로 매핑하여 모터 제어
    int motorOutputVal = map(serialInputVal, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);
    analogWrite(throttle_output_pin, motorOutputVal);


  } else {
    int pedalVal1023 = analogRead(pedal_input_pin);   // 아날로그 입력핀으로부터 페달 입력 값을 읽어옴

    int pedalConvertedVal255 = map(pedalVal1023, MIN_PEDAL_INPUT, MAX_PEDAL_INPUT_1023, 0, 255);

    // 만약 변환된 값이 255보다 크면 255로 제한
    if (pedalConvertedVal255 > 255) pedalConvertedVal255 = 255;
    // 만약 변환된 값이 0보다 작으면 0으로 제한
    if (pedalConvertedVal255 < 0) pedalConvertedVal255 = 0;

    // 변환된 값을 모터 제어
    analogWrite(throttle_output_pin, pedalConvertedVal255);

    // 페달 입력 값을 시리얼 모니터에 출력

    // sprintf(buf, "Pedal Value: %d/1023", pedalVal1023);
    Serial.print(pedalVal1023);
    Serial.print("/1023 -> Input Value / ");
    Serial.print(pedalConvertedVal255);
    Serial.println("/1023 -> converted Value");

    Serial.println(buf);
  }
}