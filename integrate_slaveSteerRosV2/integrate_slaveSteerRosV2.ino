
const int steer_step_pin = 8;
const int steer_dir_pin = 9;
const int steer_en_pin = 10;
const int switch_pin = 11;

const int MAX_STEER_L_STEP = 8000;
const int MAX_STEER_R_STEP = -8000;

float mtSpeed = 0;               // Initial speed setting (0 means stopped)
int stepDirection = 0;           // Initial stepDirection: 0 for stop, 1 for counterclockwise, -1 for clockwise
int REDUCER_GEAR_RATIO = 20;     // 감속기 기어비
float GEARBOX_RATIO = 1.5;       // 조향하우징 기어비
long totalSteps = 0;             // Total steps moved
bool isStepMotorStopped = true;  // Flag to track if the motor is currently stopped
int prevState = 0;
float steer_velocity = 0;

char serialChar;
String serialStr = "";
bool isStrRead = false;
int steerVelocityInt = 0;


void setup() {
  pinMode(steer_step_pin, OUTPUT);
  pinMode(steer_dir_pin, OUTPUT);
  pinMode(steer_en_pin, OUTPUT);
  pinMode(switch_pin, INPUT_PULLUP);

  Serial.begin(38400);     // Set the baud rate to 38400
  // Serial.setTimeout(100);  // Set a timeout for parsing input
}

void loop() {
  Serial.println("Loop "+String(Serial.available()));
  while (Serial.available() >= 4) {
    serialChar = Serial.read();
    if (serialChar == 's') {
      serialStr = "";
      isStrRead = true;
    } else if (serialChar == 'f' && isStrRead) {
      isStrRead = false;
      steerVelocityInt = (int)serialStr.toInt();
      Serial.println("spd "+String(steerVelocityInt));

      if (abs(steerVelocityInt) >= 1) {
        steer_velocity = ((float)steerVelocityInt);
      } else {
        steer_velocity = 0;
      }
      break;
    } else {
      if (isStrRead) serialStr += serialChar;
    }
  }

  if (prevState == 0 && digitalRead(switch_pin) == 1) {
    totalSteps = 0;
  }

  // if (Serial.available()>0 && digitalRead(switch_pin) == 1) {
  if (digitalRead(switch_pin) == 1) {
    // steer_velocity = steer_velocity / 100.0;
    // steer_velocity = Serial.readStringUntil('\n').toFloat();

    // Determine stepDirection based on the sign of the input
    if (steerVelocityInt < 0) {
      stepDirection = -1;  // counterclockwise for negative input
    } else if (steerVelocityInt > 0) {
      stepDirection = 1;  // clockwise for positive input
    } else {
      stepDirection = 0;  // Stop for input of 0
    }

    // 모터의 초당 이동할 스텝 수(steps/s) 를 계산 :steps per revolution, and microsteer_step_ping 이용
    mtSpeed = abs(steerVelocityInt) * REDUCER_GEAR_RATIO * GEARBOX_RATIO * 2 * 4 / (2 * PI);
    // mtSpeed = abs(steer_velocity) * REDUCER_GEAR_RATIO * GEARBOX_RATIO * 200 * 4 / (2 * PI);

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

  prevState = digitalRead(switch_pin);
}