#include <SoftwareSerial.h>

const int steer_step_pin = 9;
const int steer_dir_pin = 8;
const int steer_en_pin = 5;

const int MAX_STEER_L_STEP = 8000;
const int MAX_STEER_R_STEP = -8000;

float mtSpeed = 0;               // Initial speed setting (0 means stopped)
int stepDirection = 0;           // Initial stepDirection: 0 for stop, 1 for counterclockwise, -1 for clockwise
int REDUCER_GEAR_RATIO = 20;     // 감속기 기어비
float GEARBOX_RATIO = 1.5;       // 조향하우징 기어비
long totalSteps = 0;             // Total steps moved
bool isStepMotorStopped = true;  // Flag to track if the motor is currently stopped

SoftwareSerial SLAVESerial(5,6);

void setup() {
  SLAVESerial.begin(38400);
  pinMode(steer_step_pin, OUTPUT);
  pinMode(steer_dir_pin, OUTPUT);
  pinMode(steer_en_pin, OUTPUT);
  Serial.begin(38400);     // Set the baud rate to 38400
  Serial.setTimeout(100);  // Set a timeout for parsing input
}

void loop() {
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');  // Read the input until a newline character is encountered
    inputStr = SLAVESerial.readStringUntil('\n');
    inputStr.trim();                                 // Remove leading/trailing whitespace
    float steer_velocity = inputStr.toFloat();       // 핸들 각속도 입력받음

    // Determine stepDirection based on the sign of the input
    if (steer_velocity < 0) {
      stepDirection = -1;  // counterclockwise for negative input
    } else if (steer_velocity > 0) {
      stepDirection = 1;  // clockwise for positive input
    } else {
      stepDirection = 0;  // Stop for input of 0
    }

    // 모터의 초당 이동할 스텝 수(steps/s) 를 계산 :steps per revolution, and microsteer_step_ping 이용
    mtSpeed = abs(steer_velocity) * REDUCER_GEAR_RATIO * GEARBOX_RATIO * 200 * 4 / (2 * PI);

    Serial.print("Input (steering Velocity): ");
    Serial.println(steer_velocity);  // Print the input value to the Serial Monitor

    // If motor was previously stopped, enable the motor
    if (isStepMotorStopped) {
      digitalWrite(steer_en_pin, LOW);
      isStepMotorStopped = false;
    }
  }

  if (mtSpeed > 0 && !isStepMotorStopped) {
    digitalWrite(steer_dir_pin, stepDirection == 1 ? LOW : HIGH);  // Set stepDirection based on the 'stepDirection' variable

    // Calculate the delay based on the speed
    int step_delay = (int)(1000000 / mtSpeed);  // Calculate the delay in microseconds

    // Step signal ON
    digitalWrite(steer_step_pin, HIGH);
    delayMicroseconds(step_delay);

    // Step signal OFF
    digitalWrite(steer_step_pin, LOW);
    delayMicroseconds(step_delay);

    // Update total steps moved
    totalSteps += stepDirection;

    // Check step limits and stop if out of range
    if (totalSteps > MAX_STEER_L_STEP || totalSteps < MAX_STEER_R_STEP) {
      if (mtSpeed == 0) {
        // If the motor is already stopped, enable it again
        digitalWrite(steer_en_pin, LOW);
        isStepMotorStopped = false;
      } else {
        totalSteps = max(min(totalSteps, MAX_STEER_L_STEP), MAX_STEER_R_STEP);  // Clamp within the range
        digitalWrite(steer_en_pin, HIGH);                                       // Disable the motor to stop
        isStepMotorStopped = true;
      }
    }
  } else {
    digitalWrite(steer_en_pin, HIGH);  // Disable the motor (stop)
    isStepMotorStopped = true;
  }
}